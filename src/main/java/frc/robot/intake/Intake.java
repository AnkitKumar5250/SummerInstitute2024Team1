package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static frc.robot.Ports.Intake.pivotPort;
import static frc.robot.Ports.Intake.rollerPort;
import static frc.robot.drivetrain.DrivetrainConstants.MAXIMUM_VOLTAGE;
import static frc.robot.drivetrain.DrivetrainConstants.MINIMUM_VOLTAGE;
import static frc.robot.intake.IntakeConstants.EXTENDED_ANGLE;
import static frc.robot.intake.IntakeConstants.RETRACTED_ANGLE;
import static frc.robot.intake.IntakeConstants.TARGET_VOLTAGE;
import static frc.robot.intake.IntakeConstants.VOLTS_TO_VELOCTIY;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.intake.IntakeConstants.PivotFFD;
import frc.robot.intake.IntakeConstants.PivotPID;

/** The Intake Subsystem. */
public class Intake extends SubsystemBase {
    // Instantiates motors
    private final CANSparkMax roller = new CANSparkMax(rollerPort, MotorType.kBrushless);
    private final CANSparkMax pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);

    // Instantiates encoders
    private final AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();

    // Instantiates controllers
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
        double pidOutput = pidControllerPivot.calculate(pivotEncoder.getPosition() * 360);
        double ffdOutput = armFeedforward.calculate(pidControllerPivot.getSetpoint().position,
                pidControllerPivot.getSetpoint().velocity);
        Measure<Velocity<Distance>> velocity = MetersPerSecond.of(pidOutput + ffdOutput/2);
        Measure<Voltage> voltage = Volts
                .of(velocity.in(MetersPerSecond) * VOLTS_TO_VELOCTIY.in(VoltsPerMeterPerSecond));
        if (voltage.gt(MAXIMUM_VOLTAGE)) {
            voltage = MAXIMUM_VOLTAGE;
        }
        if (voltage.lt(MINIMUM_VOLTAGE)) {
            voltage = MINIMUM_VOLTAGE;
        }
        return voltage;
    }

    /** Constructor */
    public Intake() {
        roller.restoreFactoryDefaults();
        roller.burnFlash();
        pivot.restoreFactoryDefaults();
        pivot.burnFlash();
        pidControllerPivot.setTolerance(PivotPID.ANGLE_TOLERANCE.in(Degrees));
    }

    /**
     * Extends the intake.
     *
     * @return A command.
     */
    public Command extend() {
        return goTo(EXTENDED_ANGLE);
    }

    /**
     * Retracts the intake.
     *
     * @return A command.
     */
    public Command retract() {
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
                .andThen(run(() -> pivot.setVoltage(calculateVoltage().in(Volts))))
                .until(() -> pidControllerPivot.atSetpoint());
    }

    /**
     * Starts the rollers.
     *
     * @return A command.
     */
    public Command startRoller() {
        return runOnce(
            () -> roller.setVoltage(TARGET_VOLTAGE.in(Volts)))
                    .andThen(Commands.idle(this))
                    .finallyDo(() -> roller.set(0));
    }

    /**
     * Stops the rollers.
     *
     * @return A command.
     */
    public Command stopRoller() {
        return runOnce(() -> roller.stopMotor()).andThen(Commands.idle(this))
                .finallyDo(() -> roller.set(0));
    }

}
