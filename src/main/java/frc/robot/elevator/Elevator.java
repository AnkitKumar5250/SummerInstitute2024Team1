package frc.robot.elevator;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Ports.Elevator.elevatorPort;
import static frc.robot.Ports.Intake.beamBreakEntrancePort;
import static frc.robot.elevator.ElevatorConstants.P;
import static frc.robot.elevator.ElevatorConstants.D;
import static frc.robot.elevator.ElevatorConstants.I;

public class Elevator extends SubsystemBase {
    // Instantiates motor
    private final CANSparkMax motor = new CANSparkMax(elevatorPort, MotorType.kBrushless);

    // Instantiates beambreak
    private final DigitalInput beamBreak = new DigitalInput(beamBreakEntrancePort);

    // Instantiates PID Controller
    private final PIDController PID = new PIDController(P, I, D);

    /**
     * Constructor
     */
    public Elevator() {

    }

    /**
     * Returns the reading of the beambreak
     */
    public boolean getBeamBreak() {
        return this.beamBreak.get();
    }

    /**
     * Runs elevator.
     * 
     * @return A command.
     */
    public Command start() {
        return run(
                () -> motor
                        .setVoltage(PID.calculate(motor.get(), ElevatorConstants.TARGET_VOLTAGE.in(Volts))));
    }

    /**
     * Stops elevator.
     * 
     * @return A command.
     */
    public Command stop() {
        return runOnce(
                () -> motor
                        .setVoltage(0));
    }
}
