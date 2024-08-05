package frc.robot.elevator;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.Elevator.*;
import static frc.robot.Ports.Intake.beamBreakEntrancePort;

public class Elevator extends SubsystemBase {
    boolean elevated = false;
    BooleanSupplier isElevated = () -> elevated;

    CANSparkMax elevator = new CANSparkMax(elevatorPort, MotorType.kBrushless);
    DigitalInput beamBreak = new DigitalInput(beamBreakEntrancePort);

    public boolean getBeamBreak() {
        return this.beamBreak.get();
    }

    // Add PID pls
    public Command elevatorBrake() {
        if (beamBreak.get() == true) {
            return run(
                    () -> elevator.set(1)).finallyDo(
                            () -> elevator.set(0));
        }
        return Commands.none();
    }

    // Starts the elevator
    public Command toggleElevation() {
        return runOnce(
                () -> {
                    if (isElevated.getAsBoolean()) {
                        elevator.set(0);
                        elevated = false;
                    } else {
                        elevator.set(1);
                        elevated = true;
                    }
                });
    }

}
