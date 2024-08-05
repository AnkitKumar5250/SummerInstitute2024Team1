package frc.robot.drivetrain;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.positioning.Positioning;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.MINIMUM_VOLTAGE_THRESHHOLD;
import static frc.robot.drivetrain.DrivetrainConstants.*;

/**
 * The Drivetrain Subsystem.
 */
public class Drivetrain extends SubsystemBase {
    // Instantiates motors
    private final CANSparkMax leftLeader = new CANSparkMax(0, kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(1, kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(2, kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(3, kBrushless);

    // Instantiates Differential Drive
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    // Instantiates Odometry
    // private final DifferentialDriveOdometry diffDriveOdometry = new 

    // Instantiates encoders
    private final Encoder leftEncoder = new Encoder(leftEncoderSourceA, leftEncoderSourceB);
    private final Encoder rightEncoder = new Encoder(RightEncoderSourceA, RightEncoderSourceB);

    // Instantiates PID controllers
    private final PIDController pidControllerRotation = new PIDController(1, 0, 1);
    private final PIDController pidControllerTranslation = new PIDController(moveP, moveI, moveD);

    /**
     * Constructor.
     */
    public Drivetrain() {
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        rightLeader.burnFlash();
        rightFollower.burnFlash();
        leftLeader.burnFlash();
        leftFollower.burnFlash();
    }

    /**
     * Updates voltage based on driver input.
     * 
     * @param leftSpeed  : Y-axis of left joystick.
     * @param rightSpeed : X-axis of right joystick.
     * @return A command.
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        diffDrive.tankDrive(leftSpeed, -rightSpeed);
    }

    /**
     * Updates voltage based on PID in order to drive a certain distance.
     * 
     * @param distance distance to drive the robot.
     * @return voltage of the motors.
     */
    public void drive(Measure<Distance> distance) {
        double voltage;
        double encoderValue = Math.abs(rightEncoder.getDistance());
        voltage = pidControllerTranslation.calculate(encoderValue, distance.in(Meters));

        if (Math.abs(voltage) > 1) {
            voltage = Math.copySign(1, voltage);
        }

        leftLeader.set(voltage);
        rightLeader.set(voltage);

        Positioning.updateRobotPosition(encoderValue, false);
    }

    /**
     * Updates voltage based on PID in order to fufill rotation command.
     * 
     * @param angle : angle to rotate.
     */
    public void rotate(Measure<Angle> angle) {
        double distance = angle.in(Degrees) * TURNING_RADIUS * 2 * Math.PI / 360;

        double encoderValue = Math.abs(rightEncoder.getDistance());
        double voltage = pidControllerRotation.calculate(encoderValue, distance);

        if (Math.abs(voltage) > 1) {
            voltage = Math.copySign(1, voltage);
        }

        leftLeader.setVoltage(-voltage);
        rightLeader.setVoltage(voltage);

        Positioning.updateRobotPosition(encoderValue, true);
    }

    /**
     * Updates voltage based on PID in order to fufill rotation command.
     * 
     * @param negate : whether to rotate the opposite direction(clockwise) or not.
     * @param angle  : angle to rotate.
     */
    public void rotate(Measure<Angle> angle, boolean negate) {
        double distance = angle.in(Degrees) * TURNING_RADIUS * 2 * Math.PI / 360;

        double encoderValue = Math.abs(rightEncoder.getDistance());
        double voltage = pidControllerRotation.calculate(encoderValue, distance);

        if (Math.abs(voltage) > 1) {
            voltage = Math.copySign(1, voltage);
        }
        if (negate) {
            leftLeader.setVoltage(voltage);
            rightLeader.setVoltage(-voltage);
        } else {
            leftLeader.setVoltage(-voltage);
            rightLeader.setVoltage(voltage);
        }

        Positioning.updateRobotPosition(encoderValue, true);
    }

    /**
     * Drives based on driver input.
     * 
     * @param leftSpeed  Y-axis of left joystick.
     * @param rightSpeed X-axis of right joystick.
     * @return A command.
     */
    public Command driveCommand(double leftSpeed, double rightSpeed) {
        return Commands.run(() -> tankDrive(leftSpeed, rightSpeed));
    }

    /**
     * Drives a certain distance.
     *
     * @param meters : distance to drive.
     * @return A command.
     */
    public Command driveDistanceCommand(Measure<Distance> distance) {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() -> drive(distance))
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Rotates a certain angle.
     *
     * @param angle : angle to rotate.
     * @return A command.
     */
    public Command rotateDegreesCommand(Measure<Angle> angle) {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() -> rotate(angle))
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Rotates a certain angle.
     *
     * @param angle  : angle to rotate.
     * @param negate : whether to rotate clockwise or not.
     * @return A command.
     */
    public Command rotateDegreesCommand(Measure<Angle> angle, boolean negate) {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() -> rotate(angle))
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Faces robot towards bank.
     * 
     */
    public Command rotateTowardsBankCommand() {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() ->  rotate(Positioning.calcAngleTowardsBank()))
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Faces robot towards a certain position.
     * 
     * @param x : refrence point X.
     * @param y : refrence point Y.
     */
    public Command rotateTowardsPositionCommand(Measure<Distance> x, Measure<Distance> y) {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() ->  rotate(Positioning.calcAngleTowardsPosition(x, y)))
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

}
