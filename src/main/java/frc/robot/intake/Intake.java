package frc.robot.intake;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Import all necessary items
import static frc.robot.intake.IntakeConstants.*;
import static frc.robot.Ports.Elevator.elevatorPort;
import static frc.robot.Ports.Intake.*;

public class Intake extends SubsystemBase {
    boolean extended = false;
    BooleanSupplier isExtended = () -> extended;
    // boolean elevated = false;
    // BooleanSupplier isElevated = () -> elevated;
    AbsoluteEncoder pivotEncoder;
    CANSparkMax roller = new CANSparkMax(rollerPort, MotorType.kBrushless);
    CANSparkMax pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);
    CANSparkMax elevator = new CANSparkMax(elevatorPort, MotorType.kBrushless);
    // CANSparkMax elevator = new
    // CANSparkMax(Constants.IntakeConstants.elevatorPort, MotorType.kBrushless);
    DigitalInput beamBreak = new DigitalInput(beamBreakEntrancePort);

    public Intake() {
        pivotEncoder = pivot.getAbsoluteEncoder();
    }

    // Extends until a certain "angle" is reached
    public Command extend() {
        return runOnce(
                () -> pivot.set(.5)).andThen(
                        //
                        Commands.waitUntil(() -> pivotEncoder.getPosition() >= stopPoint)
                                .andThen(() -> pivot.set(0)));
    }

    // Retracts until the original point
    public Command retract() {
        return runOnce(
                () -> pivot.set(-.5)).andThen(
                        Commands.waitUntil(() -> pivotEncoder.getPosition() <= startPoint)
                                .andThen(() -> pivot.set(0)));

    }

    // Starts the intake
    public Command runIntake() {
        return run(
                () -> roller.set(1)).finallyDo(
                        () -> roller.set(0));
    }

    // // Starts the elevator
    // public Command toggleElevation() {
    // return runOnce(
    // () -> {
    // if (isElevated.getAsBoolean()) {
    // elevator.set(0);
    // elevated = false;
    // } else {
    // elevator.set(1);
    // elevated = true;
    // }
    // });
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
