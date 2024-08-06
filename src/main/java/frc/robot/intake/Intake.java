package frc.robot.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Ports.Intake.pivotPort;
import static frc.robot.Ports.Intake.rollerPort;
import static frc.robot.intake.IntakeConstants.D;
import static frc.robot.intake.IntakeConstants.I;
import static frc.robot.intake.IntakeConstants.P;
import static frc.robot.intake.IntakeConstants.RETRACTED_ANGLE;
import static frc.robot.intake.IntakeConstants.EXTENDED_ANGLE;

public class Intake extends SubsystemBase {

    private final CANSparkMax roller = new CANSparkMax(rollerPort, MotorType.kBrushless);
    private final CANSparkMax pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);

    private final AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();
    private final AbsoluteEncoder rollerEncoder = roller.getAbsoluteEncoder();

    private final PIDController pidController = new PIDController(P, I, D);

    /**
     * Constructor
     */
    public Intake() {

    }

    /**
     * Extends the intake.
     *
     * @return A command.
     */
    public Command extend() {
        return runOnce(
                () -> pivot
                        .setVoltage(
                                pidController.calculate(pivotEncoder.getPosition() * 360, EXTENDED_ANGLE.in(Degrees))));
    }

    /**
     * Retracts the intake.
     *
     * @return A command.
     */
    public Command retract() {
        return runOnce(
                () -> pivot.setVoltage(
                        pidController.calculate(pivotEncoder.getPosition() * 360, RETRACTED_ANGLE.in(Degrees))));

    }

    /**
     * Command that starts rollers and ends them
     *
     * @return command that starts rollers and eventually ends them
     */
    public Command startIntake() {
        return run(
                () -> roller.set(1)).finallyDo(
                        () -> roller.set(0));
    }

}
