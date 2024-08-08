package frc.robot.drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.Drive.leftFollowerID;
import static frc.robot.Ports.Drive.leftLeaderID;
import static frc.robot.Ports.Drive.rightFollowerID;
import static frc.robot.Ports.Drive.rightLeaderID;
import static frc.robot.drivetrain.DrivetrainConstants.GEARING_REDUCTION;
import static frc.robot.drivetrain.DrivetrainConstants.MAXIMUM_VOLTAGE;
import static frc.robot.drivetrain.DrivetrainConstants.MINIMUM_VOLTAGE;
import static frc.robot.drivetrain.DrivetrainConstants.MOMENT_OF_INHERTIA;
import static frc.robot.drivetrain.DrivetrainConstants.ROBOT_MASS;
import static frc.robot.drivetrain.DrivetrainConstants.STANDART_DEVIATION;
import static frc.robot.drivetrain.DrivetrainConstants.TRACK_WIDTH;
import static frc.robot.drivetrain.DrivetrainConstants.VOLTS_TO_VELOCTIY;
import static frc.robot.drivetrain.DrivetrainConstants.WHEEL_RADIUS;

import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
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

    // Instantiates simulated encoders
    private final EncoderSim leftEncoderSim = new EncoderSim(new Encoder(null, null));
    private final EncoderSim rightEncoderSim = new EncoderSim(new Encoder(null, null));

    // Instantiates controller
    private final PIDController pidController = new PIDController(PID.P, PID.I, PID.D);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FFD.S, FFD.V, FFD.A);

    // Instantiates Simulation
    public final DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), GEARING_REDUCTION,
            MOMENT_OF_INHERTIA, ROBOT_MASS, WHEEL_RADIUS.in(Meters), TRACK_WIDTH.in(Meters), STANDART_DEVIATION);

    /**
     * use when setting voltage of motor
     *
     * @param encoder
     * @param setpoint
     * @return
     */
    private final Measure<Voltage> calculateVoltage(double measurement, double setpoint) {
        double pidOutput = pidController.calculate(measurement, setpoint);
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

    /**
     * Updates simulation.
     */
    public void updateSim() {
        sim.setInputs(leftLeader.getBusVoltage(), rightLeader.getBusVoltage());
        sim.update(0.02);

        leftEncoderSim.setDistance(sim.getLeftPositionMeters());
        leftEncoderSim.setRate(sim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(sim.getRightPositionMeters());
        rightEncoderSim.setRate(sim.getRightVelocityMetersPerSecond());
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
            leftLeader.setVoltage(calculateVoltage(leftEncoder.getVelocity(), leftSpeed).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder.getVelocity(), rightSpeed).in(Volts));
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
            leftLeader.setVoltage(calculateVoltage(leftEncoder.getPosition(), distance.in(Meters)).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder.getPosition(), distance.in(Meters)).in(Volts));

            Positioning.updateRobotPosition(rightEncoder.getPosition(), false);
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
        double distance;
        if (angle.in(Degrees) < 0) {
            distance = (360 + angle.in(Degrees)) * (TRACK_WIDTH.in(Meters) / 2) * 2 * Math.PI / 360;
        } else {
            distance = angle.in(Degrees) * (TRACK_WIDTH.in(Meters) / 2) * 2 * Math.PI / 360;
        }

        return run(() -> {
            leftLeader.setVoltage(calculateVoltage(leftEncoder.getPosition(), distance).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder.getPosition(), distance).in(Volts));

            Positioning.updateRobotPosition(rightEncoder.getPosition(), true);
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
        double distance = angle.minus(Degrees.of(Positioning.robot.getRotation().getDegrees())).in(Degrees)
                * TRACK_WIDTH.in(Meters) * Math.PI / 360;

        return run(() -> {

            leftLeader.setVoltage(calculateVoltage(leftEncoder.getPosition(), distance).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder.getPosition(), distance).in(Volts));

            Positioning.updateRobotPosition(rightEncoder.getPosition(), true);
        })
                .until(pidController::atSetpoint);
    }

    /**
     * Faces robot towards bank.
     */
    public Command rotateTowardsBank() {
        double distance = Positioning.calcAngleTowardsBank().in(Degrees) * TRACK_WIDTH.in(Meters) * Math.PI
                / 360;

        return run(() -> {

            leftLeader.setVoltage(calculateVoltage(leftEncoder.getPosition(), distance).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder.getPosition(), distance).in(Volts));

            Positioning.updateRobotPosition(rightEncoder.getPosition(), true);
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
        double distance = Positioning.calcAngleTowardsPosition(x, y).in(Degrees) * TRACK_WIDTH.in(Meters)
                * Math.PI
                / 360;
        pidController.setSetpoint(distance);

        return run(() -> {

            leftLeader.setVoltage(-calculateVoltage(rightEncoder.getPosition(), distance).in(Volts));
            rightLeader.setVoltage(calculateVoltage(rightEncoder.getPosition(), distance).in(Volts));

            Positioning.updateRobotPosition(rightEncoder.getPosition(), true);
        })
                .until(pidController::atSetpoint);
    }

}
