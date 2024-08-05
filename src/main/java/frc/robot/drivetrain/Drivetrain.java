package frc.robot.drivetrain;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Vision;

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
    private final DifferentialDrive DiffDrive = new DifferentialDrive(leftLeader, rightLeader);

    // Instantiates encoders
    private final Encoder leftEncoder = new Encoder(leftEncoderSourceA, leftEncoderSourceB);
    private final Encoder rightEncoder = new Encoder(RightEncoderSourceA, RightEncoderSourceB);

    // used for tracking
    private double prevLeftEncoder;
    private double prevRightEncoder;

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
        DiffDrive.tankDrive(leftSpeed, -rightSpeed);
    }

    public double calcEncoderDifference() {
        double currEncoderValue = Math.abs(rightEncoder.get());
        double prevEncoderValue = prevLeftEncoder + prevRightEncoder / 2;

        prevLeftEncoder = Math.abs(leftEncoder.get());
        prevRightEncoder = Math.abs(rightEncoder.get());

        return currEncoderValue - prevEncoderValue;
    }

    public void updateRobotPosition(double encoderValue, boolean isRotating) {
        Rotation2d rotation = new Rotation2d();
        Translation2d translation = new Translation2d();
        Transform2d transform;
        if (isRotating) {
            rotation = new Rotation2d(encoderValue / (TURNING_RADIUS * 2 * Math.PI / 360));
        }
        if (!isRotating) {
            double xComp = Vision.Robot.getRotation().getCos() * encoderValue;
            double yComp = Vision.Robot.getRotation().getSin() * encoderValue;
            translation = new Translation2d(xComp, yComp);
        }

        transform = new Transform2d(translation, rotation);
        Vision.Robot.transformBy(transform);
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

        updateRobotPosition(encoderValue, false);
    }

    /**
     * Updates voltage based on PID in order to fufill rotation command.
     * 
     * @param angle : angle to rotate.
     */
    public void rotate(Measure<Angle> angle) {
        double distance = angle.in(Degrees) * TURNING_RADIUS * 2 * Math.PI / 360;

        double encoderValue = Math.abs(rightEncoder.getDistance());
        double voltage = pidControllerRotation.calculate(encoderValue / 2, distance);

        if (Math.abs(voltage) > 1) {
            voltage = Math.copySign(1, voltage);
        }

        leftLeader.setVoltage(-voltage);
        rightLeader.setVoltage(voltage);

        updateRobotPosition(encoderValue, true);
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
        double voltage = pidControllerRotation.calculate(encoderValue / 2, distance);

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

        updateRobotPosition(encoderValue, true);
    }

    /**
     * Updates voltage based on PID in order to fufill rotation command.
     */
    public void rotateTowardsBank() {
        rotate(Vision.calcAngleTowardsBank());
    }

    /**
     * Updates voltage based on PID in order to fufill rotation command.
     * 
     * @param x : refrence point X.
     * @param y : refrence point Y.
     */
    public void rotateTowardsPosition(Measure<Distance> x, Measure<Distance> y) {
        rotate(Vision.calcAngleTowardsPosition(x, y));
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
        return Commands.run(() -> rotate(angle))
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Faces robot towards bank.
     * 
     */
    public Command rotateTowardsBankCommand() {
        return Commands.run(() -> rotateTowardsBank())
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Faces robot towards a certain position.
     * 
     * @param x : refrence point X.
     * @param y : refrence point Y.
     */
    public Command rotateTowardsPositionCommand(Measure<Distance> x, Measure<Distance> y) {
        return Commands.run(() -> rotateTowardsPosition(x, y))
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

}
