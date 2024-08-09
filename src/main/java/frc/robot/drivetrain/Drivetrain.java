package frc.robot.drivetrain;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Ports.Drive.leftFollowerID;
import static frc.robot.Ports.Drive.leftLeaderID;
import static frc.robot.Ports.Drive.rightFollowerID;
import static frc.robot.Ports.Drive.rightLeaderID;
import static frc.robot.drivetrain.DrivetrainConstants.GEARING_RATIO;
import static frc.robot.drivetrain.DrivetrainConstants.MAXIMUM_VOLTAGE;
import static frc.robot.drivetrain.DrivetrainConstants.MINIMUM_VOLTAGE;
import static frc.robot.drivetrain.DrivetrainConstants.MOMENT_OF_INHERTIA;
import static frc.robot.drivetrain.DrivetrainConstants.ROBOT_MASS;
import static frc.robot.drivetrain.DrivetrainConstants.STANDART_DEVIATION;
import static frc.robot.drivetrain.DrivetrainConstants.TRACK_WIDTH;
import static frc.robot.drivetrain.DrivetrainConstants.WHEEL_RADIUS;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivetrain.DrivetrainConstants.FFD;
import frc.robot.drivetrain.DrivetrainConstants.PID;
import frc.robot.positioning.Positioning;

/**
 * The Drivetrain Subsystem.
 */
public class Drivetrain extends SubsystemBase {

    // Instantiates motors
    private final CANSparkMax leftLeader = new CANSparkMax(leftLeaderID, kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(leftFollowerID, kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(rightLeaderID, kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(rightFollowerID, kBrushless);

    // Instantiates encoders
    private final AbsoluteEncoder leftEncoder = leftLeader.getAbsoluteEncoder();
    private final AbsoluteEncoder rightEncoder = rightLeader.getAbsoluteEncoder();

    // Used for recording previous drivetrain position
    public static final DifferentialDriveWheelPositions previousEncoderValues = new DifferentialDriveWheelPositions(0,
            0);

    // Instantiates controller
    private final PIDController pidControllerDistance = new PIDController(PID.P, PID.I, PID.D);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FFD.S, FFD.V, FFD.A);

    // Instantiates Simulation
    public final DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), GEARING_RATIO,
            MOMENT_OF_INHERTIA, ROBOT_MASS, WHEEL_RADIUS.in(Meters), TRACK_WIDTH.in(Meters), STANDART_DEVIATION);

    /**
     * Returns encoder position
     * 
     * @return encoder position
     */
    private final Measure<Distance> getEncoderPosition() {
        return Meters.of(leftEncoder.getPosition() + rightEncoder.getPosition() / 2);
    }

    /**
     * Returns encoder displacement
     * 
     * @return encoder displacement
     */
    private final Measure<Distance> getEncoderDisplacement() {
        Measure<Distance> displacement = Meters.of((leftEncoder.getPosition() - previousEncoderValues.leftMeters)
                + (rightEncoder.getPosition() - previousEncoderValues.rightMeters) / 2);
        previousEncoderValues.leftMeters = leftEncoder.getPosition();
        previousEncoderValues.rightMeters = rightEncoder.getPosition();
        return displacement;
    }

    /**
     * Use when setting voltage of motor based on a distance setpoint.
     *
     * @param distanceSetPoint
     *            : The setpoint in terms of distance.
     * @return A voltage.
     */
    private final Measure<Voltage> calculateVoltageFromDistance(Measure<Distance> distanceSetPoint) {
        distanceSetPoint.plus(getEncoderPosition());
        double pidOutput = pidControllerDistance.calculate(getEncoderPosition().in(Meters),
                distanceSetPoint.in(Meters));
        Measure<Voltage> voltage = Volts.of(feedforward.calculate(pidOutput));
        if (voltage.gt(MAXIMUM_VOLTAGE)) {
            voltage = MAXIMUM_VOLTAGE;
        }
        if (voltage.lt(MINIMUM_VOLTAGE)) {
            voltage = MINIMUM_VOLTAGE;
        }
        return voltage;
    }

    /**
     * Use when setting voltage of motor based on a velocity setpoint.
     *
     * @param setpoint
     *            : The setpoint in terms of velocity.
     * @return A voltage.
     */
    private final Measure<Voltage> calculateVoltageFromVelocity(Measure<Velocity<Distance>> velocitySetPoint) {
        Measure<Voltage> voltage = Volts.of(feedforward.calculate(velocitySetPoint.in(MetersPerSecond)));
        if (voltage.gt(MAXIMUM_VOLTAGE)) {
            voltage = MAXIMUM_VOLTAGE;
        }
        if (voltage.lt(MINIMUM_VOLTAGE)) {
            voltage = MINIMUM_VOLTAGE;
        }
        return voltage;
    }

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

        leftLeader.setIdleMode(IdleMode.kBrake);
        rightLeader.setIdleMode(IdleMode.kBrake);
        leftFollower.setIdleMode(IdleMode.kBrake);
        rightFollower.setIdleMode(IdleMode.kBrake);

        rightLeader.burnFlash();
        rightFollower.burnFlash();
        leftLeader.burnFlash();
        leftFollower.burnFlash();

        leftEncoder.setPositionConversionFactor(WHEEL_RADIUS.in(Meters) * 2 * Math.PI);
        rightEncoder.setPositionConversionFactor(WHEEL_RADIUS.in(Meters) * 2 * Math.PI);

        leftEncoder.setVelocityConversionFactor(WHEEL_RADIUS.in(Meters) * 2 * Math.PI);
        rightEncoder.setVelocityConversionFactor(WHEEL_RADIUS.in(Meters) * 2 * Math.PI);
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
            leftLeader.setVoltage(
                    calculateVoltageFromVelocity(MetersPerSecond.of(Math.copySign(Math.pow(leftSpeed, 2), leftSpeed)))
                            .in(Volts));
            rightLeader.setVoltage(
                    calculateVoltageFromVelocity(MetersPerSecond.of(Math.copySign(Math.pow(rightSpeed, 2), rightSpeed)))
                            .in(Volts));
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
        return run(() -> {
            leftLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));
            rightLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));

            Positioning.updateRobotPosition(getEncoderDisplacement(), false);
        }).until(pidControllerDistance::atSetpoint);
    }

    /**
     * Rotates a certain angle counterclockwise.
     *
     * @param angle
     *            : angle to rotate.
     * @return A command.
     */
    public Command rotateDegrees(Measure<Angle> angle) {
        Measure<Distance> distance;

        if (angle.in(Degrees) < 0) {
            distance = Meters.of((360 + angle.in(Degrees)) * (TRACK_WIDTH.in(Meters) / 2) * 2 * Math.PI / 360);
        } else {
            distance = Meters.of(angle.in(Degrees) * (TRACK_WIDTH.in(Meters) / 2) * 2 * Math.PI / 360);
        }

        return run(() -> {
            leftLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));
            rightLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));

            Positioning.updateRobotPosition(getEncoderDisplacement(), true);
        })
                .until(pidControllerDistance::atSetpoint);
    }

    /**
     * Rotates to a certain absolute angle.
     *
     * @param angle
     *            : angle to rotate to.
     * @return A command.
     */
    public Command rotateToAngle(Measure<Angle> angle) {
        Measure<Distance> distance = Meters
                .of(angle.minus(Degrees.of(Positioning.robot.getRotation().getDegrees())).in(Degrees)
                        * TRACK_WIDTH.in(Meters) * Math.PI / 360);

        return run(() -> {

            leftLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));
            rightLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));

            Positioning.updateRobotPosition(getEncoderDisplacement(), true);
        })
                .until(pidControllerDistance::atSetpoint);
    }

    /**
     * Faces robot towards bank.
     */
    public Command rotateTowardsBank() {
        Measure<Distance> distance = Meters
                .of(Positioning.calcAngleTowardsBank().in(Degrees) * TRACK_WIDTH.in(Meters) * Math.PI
                        / 360);

        return run(() -> {

            leftLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));
            rightLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));

            Positioning.updateRobotPosition(getEncoderDisplacement(), true);
        })
                .until(pidControllerDistance::atSetpoint);
    }

    /**
     * Faces robot towards a certain position.
     *
     * @param x
     *            : refrence point X.
     * @param y
     *            : refrence point Y.
     */
    public Command rotateTowardsPosition(Translation2d position) {
        Measure<Distance> distance = Meters
                .of(Positioning.calcAngleTowardsPosition(position).in(Degrees) * TRACK_WIDTH.in(Meters)
                        * Math.PI
                        / 360);

        return run(() -> {

            leftLeader.setVoltage(-calculateVoltageFromDistance(distance).in(Volts));
            rightLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));

            Positioning.updateRobotPosition(getEncoderDisplacement(), true);
        })
                .until(pidControllerDistance::atSetpoint);
    }

}
