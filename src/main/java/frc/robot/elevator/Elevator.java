package frc.robot.elevator;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.Elevator.elevatorPort;
import static frc.robot.Ports.Intake.beamBreakEntrancePort;
import static frc.robot.elevator.ElevatorConstants.elevatorD;
import static frc.robot.elevator.ElevatorConstants.elevatorI;
import static frc.robot.elevator.ElevatorConstants.elevatorP;

public class Elevator extends SubsystemBase {
    boolean elevated = false;

    CANSparkMax elevator = new CANSparkMax(elevatorPort, MotorType.kBrushless);
    DigitalInput beamBreak = new DigitalInput(beamBreakEntrancePort);

    private final PIDController elevatorPID = new PIDController(elevatorP, elevatorI, elevatorD);

    public boolean getBeamBreak() {
        return this.beamBreak.get();
    }

    /**
     * Detects whether beambreak is triggered or not and runs the elevator accordingly 
     * @return command to run elevator 
     */
    public Command elevatorBrake() {
        if (beamBreak.get() == true) {
            return run(
                    () -> elevator.setVoltage(elevatorPID.calculate(elevator.get(), .25))).finallyDo(
                            () -> elevator.setVoltage(elevatorPID.calculate(elevator.get(), 0)));
        }
        return Commands.none();
    }
 }
