package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Ports.Intake.pivotPort;
import static frc.robot.Ports.Intake.rollerPort;
import static frc.robot.intake.IntakeConstants.EXTENDED_ANGLE;
import static frc.robot.intake.IntakeConstants.RETRACTED_ANGLE;
import static frc.robot.intake.IntakeConstants.TARGET_VOLTAGE;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
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

    // Instantiates controllers1, 
    private final ProfiledPIDController pidControllerPivot = new ProfiledPIDController(PivotPID.P, PivotPID.I, PivotPID.D, new TrapezoidProfile.Constraints(1,1));
    private final ArmFeedforward armFeedforward = new ArmFeedforward(PivotFFD.S, PivotFFD.G, PivotFFD.V,
            PivotFFD.A);

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
        return goTo(EXTENDED_ANGLE.in(Degrees));
    }

    /**
     * Retracts the intake.
     *
     * @return A command.
     */
    public Command retract() {
        return goTo(RETRACTED_ANGLE.in(Degrees));

    }
    private Command goTo(double angle){
        return runOnce(() -> pidControllerPivot.setGoal(angle))
                .andThen(run(() -> pivot.setVoltage(
                        pidControllerPivot.calculate(pivotEncoder.getPosition() * 360)
                                + armFeedforward.calculate(pidControllerPivot.getSetpoint().position,
                                                pidControllerPivot.getSetpoint().velocity))))
                                                .until(() -> pidControllerPivot.atSetpoint());
    }
    /**
     * Starts the rollers.
     *
     * @return A command.
     */
    public Command startRoller() {
        return runOnce(() -> roller.setVoltage(TARGET_VOLTAGE.in(Volts)));
    }

    /**
     * Stops the rollers.
     *
     * @return A command.
     */
    public Command stopRoller() {
        return runOnce(() -> roller.stopMotor());
    }

}
