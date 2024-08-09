package frc.robot.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Ports.Elevator.elevatorPort;
import static frc.robot.Ports.Intake.beamBreakEntrancePort;
import static frc.robot.intake.IntakeConstants.TARGET_VOLTAGE;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The Elevator Subsystem. */
public class Elevator extends SubsystemBase {
    // Instantiates motor.
    private final CANSparkMax motor = new CANSparkMax(elevatorPort, MotorType.kBrushless);

    // Instantiates beambreak.
    private final DigitalInput beamBreak = new DigitalInput(beamBreakEntrancePort);

    /** Constructor */
    public Elevator() {
        // Restores factory default configuration on motor.
        motor.restoreFactoryDefaults();

        // Writes default configuration to flash.
        motor.burnFlash();

        // Motor won't actively brake
        motor.setIdleMode(IdleMode.kCoast);
    }

    /** Returns the reading of the beambreak */
    public boolean getBeamBreak() {
        // Gets Beam Break output and returns it.
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
                        // Sets the voltage of the motor to a predefined target voltage once.
                        .andThen(Commands.idle(this))
                        // Lets the command continue running(for some reason).
                        .finallyDo(() -> motor.set(0));
        // Turns off the motor when the command has been interupted (for some reason).
    }

    /**
     * Stops elevator.
     * 
     * @return A command.
     */
    public Command stop() {
        return runOnce(() -> motor.stopMotor())
                // Stops the motor once.
                .andThen(Commands.idle(this))
                // Lets the command continue running(for some reason).
                .finallyDo(() -> motor.set(0));
        // Turns off the motor again after the command has been interupted(for some
        // reason).
    }

}
