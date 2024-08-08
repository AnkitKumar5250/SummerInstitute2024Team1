package frc.robot.drivetrain;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static frc.robot.Ports.Drive.leftEncoderSourceA;
import static frc.robot.Ports.Drive.leftEncoderSourceB;
import static frc.robot.Ports.Drive.leftFollowerID;
import static frc.robot.Ports.Drive.leftLeaderID;
import static frc.robot.Ports.Drive.rightEncoderSourceA;
import static frc.robot.Ports.Drive.rightEncoderSourceB;
import static frc.robot.Ports.Drive.rightFollowerID;
import static frc.robot.Ports.Drive.rightLeaderID;
import static frc.robot.drivetrain.DrivetrainConstants.MAXIMUM_VOLTAGE;
import static frc.robot.drivetrain.DrivetrainConstants.MINIMUM_VOLTAGE;
import static frc.robot.drivetrain.DrivetrainConstants.TURNING_RADIUS;
import static frc.robot.drivetrain.DrivetrainConstants.VOLTS_TO_VELOCTIY;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivetrain.DrivetrainConstants.PID;
import frc.robot.positioning.Positioning;

/** The Drivetrain Subsystem. */
public class Drivetrain extends SubsystemBase {
    // Instantiates motors
    private final CANSparkMax leftLeader = new CANSparkMax(leftLeaderID, kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(leftFollowerID, kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(rightLeaderID, kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(rightFollowerID, kBrushless);

    // Instantiates Differential Drive
    // private final DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    // Instantiates encoders
    private final Encoder leftEncoder = new Encoder(leftEncoderSourceA, leftEncoderSourceB);
    private final Encoder rightEncoder = new Encoder(rightEncoderSourceA, rightEncoderSourceB);

    // Instantiates controller
    private final PIDController pidController = new PIDController(PID.P, PID.I, PID.D);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

/**
 * use when setting voltage of motor
 * @param encoder
 * @param setpoint
 * @return
 */
    private final Measure<Voltage> calculateVoltage(Encoder encoder, double setpoint) {
        double pidOutput = pidController.calculate(encoder.getDistance(), setpoint);
        double ffdOutput = feedforward.calculate(pidOutput);
        Measure<Voltage> voltage = Volts.of((pidOutput * VOLTS_TO_VELOCTIY.in(VoltsPerMeterPerSecond) + ffdOutput) / 2);
        if (voltage.gt(MAXIMUM_VOLTAGE)) {
            voltage = MAXIMUM_VOLTAGE;
        }
        if (voltage.lt(MINIMUM_VOLTAGE)) {
            voltage = MINIMUM_VOLTAGE;
        }
        return voltage;
    }

    /** Constructor. */
    public Drivetrain() {
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        leftLeader.setIdleMode(IdleMode.kBrake);
        rightLeader.setIdleMode(IdleMode.kBrake);
        leftFollower.setIdleMode(IdleMode.kBrake);
        rightFollower.setIdleMode(IdleMode.kBrake);

        rightLeader.burnFlash();
        rightFollower.burnFlash();
        leftLeader.burnFlash();
        leftFollower.burnFlash();
    }
    /**
     * Drives based on driver input.
     * 
     * @param leftSpeed
     *            Y-axis of left joystick.
     * @param rightSpeed
     *            X-axis of right joystick.
     * @return A command.
     */
    public Command drive(double leftSpeed, double rightSpeed) {
        return run(() -> {
            leftLeader.setVoltage(calculateVoltage(leftEncoder, leftSpeed).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder, rightSpeed).in(Volts));
        });
    }

    /**
     * Drives a certain distance.
     *
     * @param meters
     *            : distance to drive.
     * @return A command.
     */
    public Command drive(Measure<Distance> distance) {
        leftEncoder.reset();
        rightEncoder.reset();

        pidController.setSetpoint(distance.in(Meters));

        return run(() -> {
            leftLeader.setVoltage(calculateVoltage(leftEncoder, distance.in(Meters)).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder, distance.in(Meters)).in(Volts));

            Positioning.updateRobotPosition(rightEncoder.getDistance(), false);
        }).until(pidController::atSetpoint);
    }

    /**
     * Rotates a certain angle counterclockwise.
     *
     * @param angle
     *            : angle to rotate.
     * @return A command.
     */
    public Command rotateDegrees(Measure<Angle> angle) {
        leftEncoder.reset();
        rightEncoder.reset();

        double distance;
        if (angle.in(Degrees) < 0) {
            distance = (360 + angle.in(Degrees)) * TURNING_RADIUS.in(Meters) * 2 * Math.PI / 360;
        } else {
            distance = angle.in(Degrees) * TURNING_RADIUS.in(Meters) * 2 * Math.PI / 360;
        }


        return run(() -> {
            leftLeader.setVoltage(-calculateVoltage(rightEncoder, distance).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder, distance).in(Volts));

            Positioning.updateRobotPosition(rightEncoder.getDistance(), true);
        })
                .until(pidController::atSetpoint);
    }

    /**
     * Rotates to a certain absolute angle.
     *
     * @param angle
     *            : angle to rotate to.
     * @return A command.
     */
    public Command rotateToAngle(Measure<Angle> angle) {
        leftEncoder.reset();
        rightEncoder.reset();

        double distance = angle.minus(Degrees.of(Positioning.robot.getRotation().getDegrees())).in(Degrees)
                * TURNING_RADIUS.in(Meters) * 2 * Math.PI / 360;

        return run(() -> {

            leftLeader.setVoltage(-calculateVoltage(rightEncoder, distance).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder, distance).in(Volts));

            Positioning.updateRobotPosition(rightEncoder.getDistance(), true);
        })
                .until(pidController::atSetpoint);
    }

    /** Faces robot towards bank. */
    public Command rotateTowardsBank() {
        leftEncoder.reset();
        rightEncoder.reset();

        double distance = Positioning.calcAngleTowardsBank().in(Degrees) * TURNING_RADIUS.in(Meters) * 2 * Math.PI
                / 360;

        return run(() -> {

            leftLeader.setVoltage(-calculateVoltage(rightEncoder, distance).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder, distance).in(Volts));

            Positioning.updateRobotPosition(rightEncoder.getDistance(), true);
        })
                .until(pidController::atSetpoint);
    }

    /**
     * Faces robot towards a certain position.
     * 
     * @param x
     *            : refrence point X.
     * @param y
     *            : refrence point Y.
     */
    public Command rotateTowardsPosition(Measure<Distance> x, Measure<Distance> y) {
        leftEncoder.reset();
        rightEncoder.reset();

        double distance = Positioning.calcAngleTowardsPosition(x, y).in(Degrees) * TURNING_RADIUS.in(Meters) * 2
                * Math.PI
                / 360;
        pidController.setSetpoint(distance);

        return run(() -> {

            leftLeader.setVoltage(-calculateVoltage(rightEncoder, distance).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder, distance).in(Volts));

            Positioning.updateRobotPosition(rightEncoder.getDistance(), true);
        })
                .until(pidController::atSetpoint);
    }

}
