package frc.robot.intake;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.intake.IntakeConstants.*;
import static frc.robot.Ports.Intake.*;

public class Intake extends SubsystemBase {
    boolean extended = false;
    BooleanSupplier isExtended = () -> extended;
    AbsoluteEncoder pivotEncoder;

    CANSparkMax roller = new CANSparkMax(rollerPort, MotorType.kBrushless);
    CANSparkMax pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);

    private final PIDController intakePID = new PIDController(intakeP, intakeI, intakeD);
    public Intake() {
        pivotEncoder = pivot.getAbsoluteEncoder();
    }

    // Add PID to all three of these methods pls

    // Extends until a certain "angle" is reached

    /**
     * method that extends the pivot
     * @return command to extend pivot
     */
    public Command extend() {
        return runOnce(
                () -> pivot.set(intakePID.calculate(pivot.get(), stopPoint))).andThen(
                        //
                        
                                .andThen(() -> pivot.set(intakePID.calculate(pivot.get(), 0)));
    }

    // Retracts until the original point
    public Command retract() {
        return runOnce(
                () -> pivot.set(-.5)).andThen(
                        Commands.waitUntil(() -> pivotEncoder.getPosition() <= startPoint)
                                .andThen(() -> pivot.set(0)));

    }

    // Starts the intake
    public Command startIntake() {
        return run(
                () -> roller.set(1)).finallyDo(
                        () -> roller.set(0));
    }

   
}
