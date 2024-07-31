package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Position;

import static frc.robot.Ports.Shooter.*;
import static frc.robot.shooter.ShooterConstants.*;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.*;
import static frc.robot.Constants.FieldConstants.*;


public class Shooter extends SubsystemBase {
    // Instantiate motor
    private final CANSparkMax motor = new CANSparkMax(motorPort, MotorType.kBrushless);

    // Instantiate encoder
    private final RelativeEncoder encoder = motor.getEncoder();

    // Instantiate PID Controller
    private final PIDController pid = new PIDController(kP, kI, kD);

    /**
     * Calculates the amount of power neccesary to score the cell into the bank.
     * @return The power that the ball needs to be launched at.
     */
    public double calcVelocity() {
        Measure<Distance> xDifference = Meter.of(Math.abs(Position.X.in(Meters) - TARGET_X.in(Meters)));
        Measure<Distance> yDifference = Meters.of(Math.abs(Position.X.in(Meters) - TARGET_Y.in(Meters)));

        double hDistance = Math.hypot(xDifference.in(Meters), yDifference.in(Meters));
        double vDistance = TARGET_Z.in(Inches) - SHOOTER_HEIGHT;

        double velocity = Math.sqrt((G * Math.pow(hDistance, 2) * Math.pow(1 / Math.cos(LAUNCH_ANGLE), 2))
                / (2 * (Math.tan(LAUNCH_ANGLE) * hDistance - vDistance)));
        velocity *= POWER_COEFFICIENT;
        return velocity;
    }

    /**
     * Constructor.
     */
    public Shooter() {

    }

    /**
     * Turns off the motor.
     */
    public Command turnOff() {
        return runOnce(() -> motor.stopMotor());
    }

    /**
     * Returns the velocity of the motor in RPM.
     * @return The velocity of the motor.
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Sets the velocity of the motor using PID.
     * @param velocity The target velocity of the motor.
     */
    public Command setVelocity(double velocity) {
        return run(() -> motor.setVoltage(pid.calculate(getVelocity(), velocity)))
                .until(() ->  motor.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Sets the velocity of the motor appropriate for scoring.
     */
    public Command setVelocity() {
        return run(() -> updateVelocity()).until(() ->  motor.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Updates the velocity of the motor based on PID in order to score.
     */
    public double updateVelocity() {
        double voltage = pid.calculate(getVelocity(), calcVelocity());
        motor.setVoltage(voltage);
        return voltage;
    }

}
