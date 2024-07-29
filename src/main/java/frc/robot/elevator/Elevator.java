package frc.robot.elevator;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.Elevator.*;
import static frc.robot.Ports.Intake.beamBreakEntrancePort;
import static frc.robot.Ports.Intake.pivotPort;
import static frc.robot.Ports.Intake.rollerPort;

public class Elevator extends SubsystemBase {
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
    // Starts the intake

    // Starts the elevator
    public Command runElevator() {
        return runMotor(elevator);
    }

    public boolean getBeamBreak() {
        return this.beamBreak.get();
    }

    private Command runMotor(CANSparkMax motor) {
        return run(
                () -> motor.set(1)).finallyDo(
                        () -> motor.set(0));
    }

    // if true then it is not broken
    // if false then it is broken
    // purely for intaking process
    public Command elevatorBrake() {
        if (beamBreak.get() == true) {
            return runElevator();
        }
        return Commands.none();
    }

}
