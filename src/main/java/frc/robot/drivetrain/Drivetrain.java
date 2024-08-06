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
import static frc.robot.Constants.MINIMUM_VOLTAGE_THRESHHOLD;
import static frc.robot.Ports.Drive.RightEncoderSourceA;
import static frc.robot.Ports.Drive.RightEncoderSourceB;
import static frc.robot.Ports.Drive.RightFollowerID;
import static frc.robot.Ports.Drive.RightLeaderID;
import static frc.robot.Ports.Drive.leftEncoderSourceA;
import static frc.robot.Ports.Drive.leftEncoderSourceB;
import static frc.robot.Ports.Drive.leftFollowerID;
import static frc.robot.Ports.Drive.leftLeaderID;
import static frc.robot.drivetrain.DrivetrainConstants.TURNING_RADIUS;
import static frc.robot.drivetrain.DrivetrainConstants.D;
import static frc.robot.drivetrain.DrivetrainConstants.I;
import static frc.robot.drivetrain.DrivetrainConstants.P;
import frc.robot.positioning.Positioning;

/**
 * The Drivetrain Subsystem.
 */
public class Drivetrain extends SubsystemBase {
    // Instantiates motors
    private final CANSparkMax leftLeader = new CANSparkMax(leftLeaderID, kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(leftFollowerID, kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(RightLeaderID, kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(RightFollowerID, kBrushless);

    // Instantiates Differential Drive
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    // Instantiates encoders
    private final Encoder leftEncoder = new Encoder(leftEncoderSourceA, leftEncoderSourceB);
    private final Encoder rightEncoder = new Encoder(RightEncoderSourceA, RightEncoderSourceB);

    // Instantiates PID controllers
    private final PIDController pidControllerTranslation = new PIDController(P, I, D);

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
     * Drives based on driver input.
     * 
     * @param leftSpeed  Y-axis of left joystick.
     * @param rightSpeed X-axis of right joystick.
     * @return A command.
     */
    public Command drive(double leftSpeed, double rightSpeed) {
        return Commands.run(() -> diffDrive.tankDrive(leftSpeed, -rightSpeed));
    }

    /**
     * Drives a certain distance.
     *
     * @param meters : distance to drive.
     * @return A command.
     */
    public Command drive(Measure<Distance> distance) {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() -> {
            double voltage;
            double encoderValue = Math.abs(rightEncoder.getDistance());
            voltage = pidControllerTranslation.calculate(encoderValue, distance.in(Meters));

            if (Math.abs(voltage) > 1) {
                voltage = Math.copySign(1, voltage);
            }

            leftLeader.set(voltage);
            rightLeader.set(voltage);

            Positioning.updateRobotPosition(encoderValue, false);
        }).until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Rotates a certain angle counterclockwise.
     *
     * @param angle  : angle to rotate.
     * @param negate : whether to rotate clockwise or not.
     * @return A command.
     */
    public Command rotateDegrees(Measure<Angle> angle, boolean negate) {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() -> {
            double distance = angle.in(Degrees) * TURNING_RADIUS.in(Meters) * 2 * Math.PI / 360;

            double encoderValue = Math.abs(rightEncoder.getDistance());
            double voltage = pidControllerTranslation.calculate(encoderValue, distance);

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
        })
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Rotates a certain angle counterclockwise.
     *
     * @param angle : angle to rotate.
     * @return A command.
     */
    public Command rotateDegrees(Measure<Angle> angle) {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() -> {
            double distance = angle.in(Degrees) * TURNING_RADIUS.in(Meters) * 2 * Math.PI / 360;

            double encoderValue = Math.abs(rightEncoder.getDistance());
            double voltage = pidControllerTranslation.calculate(encoderValue, distance);

            if (Math.abs(voltage) > 1) {
                voltage = Math.copySign(1, voltage);
            }

            leftLeader.setVoltage(-voltage);
            rightLeader.setVoltage(voltage);

            Positioning.updateRobotPosition(encoderValue, true);
        })
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Rotates to a certain absolute angle.
     *
     * @param angle : angle to rotate to.
     * @return A command.
     */
    public Command rotateToAngle(Measure<Angle> angle) {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() -> {
            double distance = angle.minus(Degrees.of(Positioning.robot.getRotation().getDegrees())).in(Degrees)
                    * TURNING_RADIUS.in(Meters) * 2 * Math.PI / 360;

            double encoderValue = Math.abs(rightEncoder.getDistance());
            double voltage = pidControllerTranslation.calculate(encoderValue, distance);

            if (Math.abs(voltage) > 1) {
                voltage = Math.copySign(1, voltage);
            }

            leftLeader.setVoltage(-voltage);
            rightLeader.setVoltage(voltage);

            Positioning.updateRobotPosition(encoderValue, true);
        })
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Faces robot towards bank.
     * 
     */
    public Command rotateTowardsBank() {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() -> {
            double distance = Positioning.calcAngleTowardsBank().in(Degrees) * TURNING_RADIUS.in(Meters) * 2 * Math.PI
                    / 360;

            double encoderValue = Math.abs(rightEncoder.getDistance());
            double voltage = pidControllerTranslation.calculate(encoderValue, distance);

            if (Math.abs(voltage) > 1) {
                voltage = Math.copySign(1, voltage);
            }

            leftLeader.setVoltage(voltage);
            rightLeader.setVoltage(-voltage);

            leftLeader.setVoltage(-voltage);
            rightLeader.setVoltage(voltage);

            Positioning.updateRobotPosition(encoderValue, true);
        })
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Faces robot towards a certain position.
     * 
     * @param x : refrence point X.
     * @param y : refrence point Y.
     */
    public Command rotateTowardsPosition(Measure<Distance> x, Measure<Distance> y) {
        leftEncoder.reset();
        rightEncoder.reset();

        return Commands.run(() -> {
            double distance = Positioning.calcAngleTowardsPosition(x, y).in(Degrees) * TURNING_RADIUS.in(Meters) * 2
                    * Math.PI
                    / 360;

            double encoderValue = Math.abs(rightEncoder.getDistance());
            double voltage = pidControllerTranslation.calculate(encoderValue, distance);

            if (Math.abs(voltage) > 1) {
                voltage = Math.copySign(1, voltage);
            }

            leftLeader.setVoltage(voltage);
            rightLeader.setVoltage(-voltage);

            leftLeader.setVoltage(-voltage);
            rightLeader.setVoltage(voltage);

            Positioning.updateRobotPosition(encoderValue, true);
        })
                .until(() -> leftLeader.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

}
