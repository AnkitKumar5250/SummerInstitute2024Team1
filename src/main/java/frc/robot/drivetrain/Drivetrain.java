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
import static frc.robot.drivetrain.DrivetrainConstants.MAXIMUM_VOLTAGE;
import static frc.robot.drivetrain.DrivetrainConstants.MINIMUM_VOLTAGE;
import static frc.robot.drivetrain.DrivetrainConstants.TRACK_WIDTH;
import static frc.robot.drivetrain.DrivetrainConstants.VELOCITY_COEFFICIENT;
import static frc.robot.drivetrain.DrivetrainConstants.WHEEL_RADIUS;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivetrain.DrivetrainConstants.FFD;
import frc.robot.drivetrain.DrivetrainConstants.PID;
import frc.robot.positioning.Positioning;

/**
 * The Drivetrain Subsystem.
 */
public class Drivetrain extends SubsystemBase {

    // Instantiates motors.
    private final CANSparkMax leftLeader = new CANSparkMax(leftLeaderID, kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(leftFollowerID, kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(rightLeaderID, kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(rightFollowerID, kBrushless);

    // Instantiates encoders.
    private final AbsoluteEncoder leftEncoder = leftLeader.getAbsoluteEncoder();
    private final AbsoluteEncoder rightEncoder = rightLeader.getAbsoluteEncoder();

    // Used for recording drivetrain position at previous tick.
    public final DifferentialDriveWheelPositions previousTickEncoderValues = new DifferentialDriveWheelPositions(
            0,
            0);

    // Used for recording drivetrain position at previous command.
    public final DifferentialDriveWheelPositions previousCommandEncoderValues = new DifferentialDriveWheelPositions(
            0,
            0);

    // Instantiates controller.
    private final PIDController pidControllerDistance = new PIDController(PID.P, PID.I, PID.D);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FFD.S, FFD.V, FFD.A);

    /**
     * Sets the command encoder positions to the current encoder position.
     */
    private final void setPreviousEncoderPosition() {
        previousCommandEncoderValues.leftMeters = leftEncoder.getPosition();
        previousCommandEncoderValues.rightMeters = rightEncoder.getPosition();
    }

    /**
     * Returns the encoder position.
     * 
     * @return encoder position.
     */
    private final Measure<Distance> getEncoderPosition() {
        return Meters.of((rightEncoder.getPosition() + leftEncoder.getPosition() / 2));
    }

    /**
     * Returns encoder displacement since the last tick.
     * 
     * @return encoder displacement since the last tick.
     */
    private final Measure<Distance> getEncoderTickDisplacement() {
        Measure<Distance> displacement = Meters.of((leftEncoder.getPosition() - previousTickEncoderValues.leftMeters)
                + (rightEncoder.getPosition() - previousTickEncoderValues.rightMeters) / 2);
        previousTickEncoderValues.leftMeters = leftEncoder.getPosition();
        previousTickEncoderValues.rightMeters = rightEncoder.getPosition();
        return displacement;
    }

    /**
     * Returns encoder displacement since the last command call.
     * 
     * @return encoder displacement since the last command call.
     */
    private final Measure<Distance> getEncoderCommandDisplacement() {
        Measure<Distance> displacement = Meters.of((leftEncoder.getPosition() - previousCommandEncoderValues.leftMeters)
                + (rightEncoder.getPosition() - previousCommandEncoderValues.rightMeters) / 2);
        return displacement;
    }

    /**
     * Checks if the robot has reached the setpoint or not.
     * 
     * @param distanceSetPoint
     *            : Current distance setpoint.
     * @return True if yes, False, if no
     */
    private final boolean atDistanceSetpoint(Measure<Distance> distanceSetPoint) {
        return Math.abs(getEncoderPosition().minus(getEncoderCommandDisplacement()).in(Meters)) < pidControllerDistance
                .getPositionTolerance();
    }

    /**
     * Use when setting voltage of motor based on a distance setpoint.
     *
     * @param distanceSetPoint
     *            : The setpoint in terms of distance.
     * @return A voltage.
     */
    private final Measure<Voltage> calculateVoltageFromDistance(Measure<Distance> distanceSetPoint) {
        distanceSetPoint.plus(getEncoderCommandDisplacement());
        double pidOutput = pidControllerDistance.calculate(getEncoderCommandDisplacement().in(Meters),
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
        Measure<Voltage> voltage = Volts
                .of(feedforward.calculate(velocitySetPoint.in(MetersPerSecond)) * VELOCITY_COEFFICIENT);
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
        // Restores factory default configuration on all motors.
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        // Sets the followers and leaders.
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // Sets the idle mode to brake(robot will immideatley stop when not commanded to
        // do anything).
        leftLeader.setIdleMode(IdleMode.kBrake);
        rightLeader.setIdleMode(IdleMode.kBrake);
        leftFollower.setIdleMode(IdleMode.kBrake);
        rightFollower.setIdleMode(IdleMode.kBrake);

        // Writes configuration to flash.
        rightLeader.burnFlash();
        rightFollower.burnFlash();
        leftLeader.burnFlash();
        leftFollower.burnFlash();

        // Converts encoder readings from native unit(rotations) to meters.
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
            // Sets the voltage of the motors based on velocity given through joystick
            // input.
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
        return runOnce(() -> setPreviousEncoderPosition())
                // Records the pre-command encoder positions in order to help track progress.
                .andThen(run(() -> {
                    // Updates voltage of the motors based on distance needed to travel.
                    leftLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));
                    rightLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));

                    // Updates robot pose to match current position.
                    Positioning.updateRobotPosition(getEncoderTickDisplacement(), false);
                }).until(() -> atDistanceSetpoint(distance)));
    }

    /**
     * Rotates a certain angle counterclockwise.
     *
     * @param angle
     *            : angle to rotate.
     * @return A command.
     */
    public Command rotateDegrees(Measure<Angle> angle) {
        // Distance each wheel needs to travel in order to rotate a certain angle.
        Measure<Distance> distance;

        // If the angle is negative, add 360 to simulate a clockwise rotation(even
        // though it is rotating counterclockwise).
        if (angle.in(Degrees) < 0) {
            distance = Meters.of((360 + angle.in(Degrees)) * (TRACK_WIDTH.in(Meters) / 2) * 2 * Math.PI / 360);
        } else {
            distance = Meters.of(angle.in(Degrees) * (TRACK_WIDTH.in(Meters) / 2) * 2 * Math.PI / 360);
        }

        return runOnce(() -> setPreviousEncoderPosition())
                // Records the pre-command encoder positions in order to help track progress.
                .andThen(run(() -> {
                    // Updates voltage of the motors based on distance needed to travel.
                    leftLeader.setVoltage(-calculateVoltageFromDistance(distance).in(Volts));
                    rightLeader.setVoltage(calculateVoltageFromDistance(distance).in(Volts));

                    // Updates robot pose to match current position.
                    Positioning.updateRobotPosition(getEncoderTickDisplacement(), false);
                }).until(() -> atDistanceSetpoint(distance)));
    }

    /**
     * Rotates to a certain absolute angle(counterclockwise, 0 degrees faces
     * endzone).
     *
     * @param angle
     *            : angle to rotate to.
     * @return A command.
     */
    public Command rotateToAngle(Measure<Angle> angle) {
        // Accounts for robots current orientation.
        return rotateDegrees(angle.minus(Positioning.getOrientation()));
    }

    /**
     * Faces robot towards bank.
     */
    public Command rotateTowardsBank() {
        return rotateToAngle(Positioning.calcAngleTowardsBank());
    }

    /**
     * Faces robot towards a certain position.
     *
     * @param translation
     *            : translation representing a coordinate.
     */
    public Command rotateTowardsPosition(Translation2d position) {
        return rotateToAngle(Positioning.calcAngleTowardsPosition(position));
    }

}
