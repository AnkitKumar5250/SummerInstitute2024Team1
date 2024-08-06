package frc.robot.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.Intake.pivotPort;
import static frc.robot.Ports.Intake.rollerPort;
import static frc.robot.intake.IntakeConstants.intakeD;
import static frc.robot.intake.IntakeConstants.intakeI;
import static frc.robot.intake.IntakeConstants.intakeP;
import static frc.robot.intake.IntakeConstants.startPoint;
import static frc.robot.intake.IntakeConstants.stopPoint;

public class Intake extends SubsystemBase {

    AbsoluteEncoder pivotEncoder;

    CANSparkMax roller = new CANSparkMax(rollerPort, MotorType.kBrushless);
    CANSparkMax pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);

    private final PIDController intakePID = new PIDController(intakeP, intakeI, intakeD);

    public Intake() {
        pivotEncoder = pivot.getAbsoluteEncoder();
    }

    /**
     * method that extends the pivot
     *
     * @return command to extend pivot
     */
    public Command extend() {
        return runOnce(
                () -> pivot.setVoltage(intakePID.calculate(pivotEncoder.getPosition() * 360, stopPoint)));
    }

    /**
     * method that retracts the pivot
     *
     * @return command to retract pivot
     */
    public Command retract() {
        return runOnce(
                () -> pivot.setVoltage(intakePID.calculate(pivotEncoder.getPosition() * 360, startPoint)));

    }

    /**
     * command that starts rollers and ends them
     *
     * @return command that starts rollers and eventually ends them
     */
    public Command startIntake() {
        return run(
                () -> roller.set(1)).finallyDo(
                        () -> roller.set(0));
    }

}
