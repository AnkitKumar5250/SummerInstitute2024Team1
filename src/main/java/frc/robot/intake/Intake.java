package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Ports.Intake.pivotPort;
import static frc.robot.Ports.Intake.rollerPort;
import static frc.robot.drivetrain.DrivetrainConstants.MAXIMUM_VOLTAGE;
import static frc.robot.drivetrain.DrivetrainConstants.MINIMUM_VOLTAGE;
import static frc.robot.intake.IntakeConstants.ENCODER_CONVERSION_FACTOR;
import static frc.robot.intake.IntakeConstants.EXTENDED_ANGLE;
import static frc.robot.intake.IntakeConstants.RETRACTED_ANGLE;
import static frc.robot.intake.IntakeConstants.TARGET_VOLTAGE;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.intake.IntakeConstants.PivotFFD;
import frc.robot.intake.IntakeConstants.PivotPID;

/** The Intake Subsystem. */
public class Intake extends SubsystemBase {
    // Instantiates motors.
    private final CANSparkMax roller = new CANSparkMax(rollerPort, MotorType.kBrushless);
    private final CANSparkMax pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);

    // Instantiates encoders.
    private final AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();

    // Instantiates controllers.
    private final ProfiledPIDController pidControllerPivot = new ProfiledPIDController(PivotPID.P, PivotPID.I,
            PivotPID.D, new TrapezoidProfile.Constraints(1, 1));
    private final ArmFeedforward armFeedforward = new ArmFeedforward(PivotFFD.S, PivotFFD.G, PivotFFD.V,
            PivotFFD.A);

    /**
     * Uses PID and FFD output to calculate velocity, and then converts that to
     * voltage.
     * 
     * @return voltage.
     */

    private final Measure<Voltage> calculateVoltage() {
        // Calculates PID output(velocity) based on position setpoint.
        double pidOutput = pidControllerPivot.calculate(pivotEncoder.getPosition() * 360);

        // Calculates FFD output(voltage) based on position setpoint and velocity
        // setpoint(calculated by PID).
        double ffdOutput = armFeedforward.calculate(pidControllerPivot.getSetpoint().position,
                pidOutput);

        // Voltage to be fed into the motors(before clamping)
        Measure<Voltage> voltage = Volts.of(ffdOutput);

        // Clamps voltage value to a certain range in order to avoid exccesive voltage
        // feed.
        if (voltage.gt(MAXIMUM_VOLTAGE)) {
            voltage = MAXIMUM_VOLTAGE;
        }
        if (voltage.lt(MINIMUM_VOLTAGE)) {
            voltage = MINIMUM_VOLTAGE;
        }

        // Returns final voltage to be fed into the motors.
        return voltage;
    }

    /** Constructor */
    public Intake() {
        // Restores factory default configuration on motors.
        roller.restoreFactoryDefaults();
        pivot.restoreFactoryDefaults();

        // Writes default configuration to flash
        roller.burnFlash();
        pivot.burnFlash();

        // Sets the tolerance(in degrees) of the PID controller to a constant.
        pidControllerPivot.setTolerance(PivotPID.ANGLE_TOLERANCE.in(Degrees));

        // Sets the conversion factor between native units(rotations and rotations/s) to
        // meters and meters/s
        pivotEncoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR.in(Meters));
        pivotEncoder.setVelocityConversionFactor(ENCODER_CONVERSION_FACTOR.in(Meters));
    }

    /**
     * Extends the intake.
     *
     * @return A command.
     */
    public Command extend() {
        // Moves to the predefined extended angle.
        return goTo(EXTENDED_ANGLE);
    }

    /**
     * Retracts the intake.
     *
     * @return A command.
     */
    public Command retract() {
        // Moves to the predefined retracted angle.
        return goTo(RETRACTED_ANGLE);

    }

    /**
     * Moves the pivot to a certain angle.
     * 
     * @param angle
     *            : angle to move the pivot to.
     * @return A command.
     */
    private Command goTo(Measure<Angle> angle) {
        return runOnce(() -> pidControllerPivot.setGoal(angle.in(Degrees)))
                // Sets the PID controller setpoint to the goal angle(input) once.
                .andThen(run(() -> pivot.setVoltage(calculateVoltage().in(Volts))))
                // Updates voltage of the motor periodically after setpoint has been set.
                .until(() -> pidControllerPivot.atSetpoint());
        // Stops the command once the designated angle is reached(within tolerance).
    }

    /**
     * Starts the rollers.
     *
     * @return A command.
     */
    public Command startRoller() {
        return runOnce(
                () -> roller.setVoltage(TARGET_VOLTAGE.in(Volts)))
                        // Sets the voltage of the motor to the predefined target voltage once.
                        .andThen(Commands.idle(this))
                        // Lets the command continue running(for some reason).
                        .finallyDo(() -> roller.set(0));
        // Once the command has been interupted the motor will also stop(for some
        // reason).
    }

    /**
     * Stops the rollers.
     *
     * @return A command.
     */
    public Command stopRoller() {
        return runOnce(
                () -> roller.stopMotor())
                        // Turns off the motor(once)
                        .andThen(Commands.idle(this))
                        // Lets the command continue running(for some reason)
                        .finallyDo(() -> roller.set(0));
        // Once the command has been interupted the motor will stop again(for some
        // reason).
    }

}
