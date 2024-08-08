package frc.robot.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.G;
import static frc.robot.Ports.Shooter.motorPort;
import static frc.robot.positioning.PositioningConstants.TARGET;
import static frc.robot.shooter.ShooterConstants.LAUNCH_ANGLE;
import static frc.robot.shooter.ShooterConstants.POWER_COEFFICIENT;
import static frc.robot.shooter.ShooterConstants.SHOOTER_HEIGHT;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.positioning.Positioning;
import frc.robot.shooter.ShooterConstants.PID;

/** The Shooter Subsystem. */
public class Shooter extends SubsystemBase {
    // Instantiates motor
    private final CANSparkMax motor = new CANSparkMax(motorPort, MotorType.kBrushless);

    // Instantiates encoder
    private final RelativeEncoder encoder = motor.getEncoder();

    // Instantiates pidController controller
    private final PIDController pidController = new PIDController(PID.kP, PID.kI, PID.kD);

    /**
     * Constructor.
     */
    public Shooter() {
        // inverted motor(testing motor resulted in ball launched opposite direction)
        motor.restoreFactoryDefaults();
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kCoast);
        motor.burnFlash();

        // sets the velocity tolerance of the pid controller
        pidController.setTolerance(PID.VELOCITY_TOLERANCE.in(MetersPerSecond));
    }

    /**
     * Turns off the motor.
     * 
     * @return A command.
     */
    public Command turnOff() {
        return runOnce(() -> motor.stopMotor());
    }

    /**
     * Sets the velocity of the motor appropriate for scoring.
     * 
     * @return A command.
     */
    public Command setVelocity() {
        Measure<Distance> xDifference = Meter.of(Math.abs(Positioning.robot.getX() - TARGET.getX()));
        Measure<Distance> yDifference = Meters.of(Math.abs(Positioning.robot.getY() - TARGET.getY()));

        Measure<Distance> hDistance = Meters.of(Math.hypot(xDifference.in(Meters), yDifference.in(Meters)));
        Measure<Distance> vDistance = Meters.of(TARGET.getZ() - SHOOTER_HEIGHT.in(Meters));

        double velocity = Math.sqrt((G.in(MetersPerSecond) * Math.pow(hDistance.in(Meters), 2)));

        velocity *= Math.pow(1 / Math.cos(LAUNCH_ANGLE.in(Degrees)), 2);
        velocity /= (2 * (Math.tan(LAUNCH_ANGLE.in(Degrees)) * hDistance.in(Meters) - vDistance.in(Meters)));

        final double fVelocity = velocity * POWER_COEFFICIENT;

        return runOnce(() -> pidController.setSetpoint(fVelocity))
                .andThen(run(() -> motor.setVoltage(pidController.calculate(encoder.getVelocity())))
                        .until(() -> pidController.atSetpoint()));
    }
}
