package frc.robot.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.G;
import static frc.robot.Constants.FieldConstants.TARGET;
import static frc.robot.Constants.MINIMUM_VOLTAGE_THRESHHOLD;
import static frc.robot.Ports.Shooter.motorPort;
import frc.robot.positioning.Positioning;
import static frc.robot.shooter.ShooterConstants.LAUNCH_ANGLE;
import static frc.robot.shooter.ShooterConstants.POWER_COEFFICIENT;
import static frc.robot.shooter.ShooterConstants.SHOOTER_HEIGHT;
import static frc.robot.shooter.ShooterConstants.kD;
import static frc.robot.shooter.ShooterConstants.kI;
import static frc.robot.shooter.ShooterConstants.kP;

public class Shooter extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final PIDController pid = new PIDController(kP, kI, kD);

    /**
     * Calculates the amount of power neccesary to score the cell into the bank.
     * 
     * @return The power that the ball needs to be launched at.
     */
    public double calcVelocity() {
        Measure<Distance> xDifference = Meter.of(Math.abs(Positioning.robot.getX() - TARGET.getX()));
        Measure<Distance> yDifference = Meters.of(Math.abs(Positioning.robot.getY() - TARGET.getY()));

        Measure<Distance> hDistance = Meters.of(Math.hypot(xDifference.in(Meters), yDifference.in(Meters)));
        Measure<Distance> vDistance = Meters.of(TARGET.getZ() - SHOOTER_HEIGHT.in(Meters));

        double velocity = Math
                .sqrt((G.in(MetersPerSecond) * Math.pow(hDistance.in(Meters), 2)
                        * Math.pow(1 / Math.cos(LAUNCH_ANGLE.in(Degrees)), 2))
                        / (2 * (Math.tan(LAUNCH_ANGLE.in(Degrees)) * hDistance.in(Meters) - vDistance.in(Meters))));
        velocity *= POWER_COEFFICIENT;
        return velocity;
    }

    /**
     * Constructor.
     */
    public Shooter() {
        motor.setInverted(true);
    }

    /**
     * Returns the velocity of the motor in RPM.
     * 
     * @return The velocity of the motor.
     */
    private double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Turns off the motor.
     */
    public Command turnOff() {
        return runOnce(() -> motor.stopMotor());
    }

    /**
     * Sets the velocity of the motor using PID.
     * 
     * @param velocity The target velocity of the motor.
     */
    public Command setVelocity(Measure<Velocity<Distance>> velocity) {
        return run(() -> motor.setVoltage(pid.calculate(getVelocity(), velocity.in(MetersPerSecond))))
                .until(() -> motor.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }

    /**
     * Sets the velocity of the motor appropriate for scoring.
     */
    public Command setVelocity() {
        return run(() -> motor.setVoltage(pid.calculate(getVelocity(), calcVelocity())))
                .until(() -> motor.getBusVoltage() < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts));
    }
}
