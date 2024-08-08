package frc.robot.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Ports.Elevator.elevatorPort;
import static frc.robot.Ports.Intake.beamBreakEntrancePort;
import static frc.robot.intake.IntakeConstants.TARGET_VOLTAGE;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The Elevator Subsystem. */
public class Elevator extends SubsystemBase {
    // Instantiates motor
    private final CANSparkMax motor = new CANSparkMax(elevatorPort, MotorType.kBrushless);

    // Instantiates beambreak
    private final DigitalInput beamBreak = new DigitalInput(beamBreakEntrancePort);

    /** Constructor */
    public Elevator() {

    }

    /** Returns the reading of the beambreak */
    public boolean getBeamBreak() {
        return this.beamBreak.get();
    }

    /**
     * Runs elevator.
     * 
     * @return A command.
     */
    public Command start() {
        return runOnce(
                () -> motor.setVoltage(TARGET_VOLTAGE.in(Volts)))
                        .andThen(Commands.idle(this))
                        .finallyDo(() -> motor.set(0));
    }

    /**
     * Stops elevator.
     * 
     * @return A command.
     */
    public Command stop() {
        return runOnce(() -> motor.stopMotor()).andThen(Commands.idle(this))
                .finallyDo(() -> motor.set(0));
    }

}
