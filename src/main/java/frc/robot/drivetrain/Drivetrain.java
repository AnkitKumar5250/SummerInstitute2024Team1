package frc.robot.drivetrain;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Position;
import static frc.robot.Constants.*;

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
     * @param ySpeed ...
     * @param zSpeed ...
     */
    public void arcadeDrive(double ySpeed, double zSpeed) {
        DiffDrive.arcadeDrive(ySpeed, zSpeed, true);
    }

    /**
     * Updates voltage based on driver input.
     * 
     * @param leftSpeed  : Y-axis of left joystick.
     * @param rightSpeed : X-axis of right joystick.
     * @return A command.
     */
    public void tankdrive(double leftSpeed, double rightSpeed) {
        DiffDrive.tankDrive(leftSpeed, -rightSpeed);
    }

    /**
     * Updates voltage based on PID in order to drive a certain distance.
     * 
     * @param distance distance to drive the robot.
     * @return voltage of the motors.
     */
    public void driveDistance(Measure<Distance> distance) {
        double voltage = 0;
        double encoderValue = leftEncoder.get() + rightEncoder.get() / 2;
        voltage = pidControllerTranslation.calculate(encoderValue, distance.in(Meters));

        if (Math.abs(voltage) > 1) {
            voltage = Math.copySign(1, voltage);
        }

        leftLeader.set(voltage);
        rightLeader.set(voltage);
    }

    /**
     * Updates voltage based on PID in order to fufill rotation command.
     * 
     * @param angle : angle to rotate.
     */
    public void rotateDegrees(Measure<Angle> angle) {
        double distance = angle.in(Degrees) * TURNING_RADIUS * 2 * Math.PI / 360;

        double encoderValue = leftEncoder.get() + rightEncoder.get() / 2;
        double voltage = pidControllerRotation.calculate(encoderValue / 2, distance);

        if (Math.abs(voltage) > 1) {
            voltage = Math.copySign(1, voltage);
        }

        leftLeader.setVoltage(-voltage);
        rightLeader.setVoltage(voltage);
    }

    /**
     * Updates voltage based on PID in order to fufill rotation command.
     */
    public void rotateTowardsBank() {
        rotateDegrees(Position.calcAngleTowardsBank());
    }

    /**
     * Updates voltage based on PID in order to fufill rotation command.
     * 
     * @param x : refrence point X.
     * @param y : refrence point Y.
     */
    public void rotateTowardsPosition(Measure<Distance> x, Measure<Distance> y) {
        rotateDegrees(Position.calcAngleTowardsPosition(x, y));
    }

    public void updatePosition(boolean rotating) {
        
    }

    /**
     * Drives based on driver input.
     * 
     * @param leftSpeed  Y-axis of left joystick.
     * @param rightSpeed X-axis of right joystick.
     * @return A command.
     */
    public Command driveCommand(double leftSpeed, double rightSpeed) {
        return Commands.run(() -> arcadeDrive(leftSpeed, rightSpeed));
    }

    /**
     * Drives a certain distance.
     *
     * @param meters : distance to drive.
     * @return A command.
     */
    public Command driveDistanceCommand(Measure<Distance> distance) {
        return Commands.run(() -> driveDistance(distance))
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Rotates a certain angle.
     *
     * @param angle : angle to rotate.
     * @return A command.
     */
    public Command rotateDegreesCommand(Measure<Angle> angle) {
        return Commands.run(() -> rotateDegrees(angle))
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Rotates a certain angle.
     *
     * @param angle : angle to rotate.
     * @return A command.
     */
    public Command rotateDegreesCommand(Measure<Angle> angle, boolean negate) {
        if (negate) {
            angle = Degrees.of(-angle.in(Degrees));
        }
        Measure<Angle> nAngle = angle;

        return Commands.run(() -> rotateDegrees(nAngle))
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
