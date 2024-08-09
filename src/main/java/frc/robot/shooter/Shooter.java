package frc.robot.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Ports.Shooter.motorPort;
import static frc.robot.positioning.PositioningConstants.TARGET;
import static frc.robot.shooter.ShooterConstants.G;
import static frc.robot.shooter.ShooterConstants.LAUNCH_ANGLE;
import static frc.robot.shooter.ShooterConstants.MAXIMUM_VOLTAGE;
import static frc.robot.shooter.ShooterConstants.MINIMUM_VOLTAGE;
import static frc.robot.shooter.ShooterConstants.POWER_COEFFICIENT;
import static frc.robot.shooter.ShooterConstants.SHOOTER_HEIGHT;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.positioning.Positioning;
import frc.robot.shooter.ShooterConstants.FFD;
import frc.robot.shooter.ShooterConstants.PID;

/** The Shooter Subsystem. */
public class Shooter extends SubsystemBase {
    // Instantiates motor.
    private final CANSparkMax motor = new CANSparkMax(motorPort, MotorType.kBrushless);

    // Instantiates encoder.
    private final RelativeEncoder encoder = motor.getEncoder();

    // Instantiates controllers(neccesary because this is a static shooter).
    private final PIDController pidController = new PIDController(PID.P, PID.I, PID.D);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FFD.S, FFD.V, FFD.A);

    /**
     * Uses PID and FFD output to calculate velocity, and then converts that to
     * voltage.
     * 
     * @return voltage.
     */

    private final Measure<Voltage> calculateVoltage() {
        // Calculates output based on velocity setpoint(returns voltage).
        double pidOutput = pidController.calculate(encoder.getVelocity());

        // Calculates output based on velocity setpoint(returns voltage).
        double ffdOutput = feedforward.calculate(pidController.getSetpoint());

        // Averages PID and FFD output.
        Measure<Voltage> voltage = Volts.of(pidOutput + ffdOutput / 2);

        // Clamps voltage value in order to avoid overfeading.
        if (voltage.gt(MAXIMUM_VOLTAGE)) {
            voltage = MAXIMUM_VOLTAGE;
        }
        if (voltage.lt(MINIMUM_VOLTAGE)) {
            voltage = MINIMUM_VOLTAGE;
        }

        // Returns voltage to be fed into the motors
        return voltage;
    }

    /**
     * Constructor.
     */
    public Shooter() {
        // Restores factory default configuration on motor.
        motor.restoreFactoryDefaults();

        // Writes factory default onfiguration to flash.
        motor.burnFlash();

        // Inverts motor(testing motor resulted in ball launched opposite direction).
        motor.setInverted(true);

        // Motor won't actively brake.
        motor.setIdleMode(IdleMode.kCoast);

        // Sets the velocity tolerance of the pid controller(meters per second).
        pidController.setTolerance(PID.VELOCITY_TOLERANCE.in(MetersPerSecond));
    }

    /**
     * Turns off the motor.
     * 
     * @return A command.
     */
    public Command turnOff() {
        return runOnce(() -> motor.stopMotor())
                // Stops the motor once.
                .andThen(Commands.idle(this))
                // Lets the command continue running(for some reason).
                .finallyDo(() -> motor.set(0));
        // Stops the motor once the command is interupted(for some reason).
    }

    /**
     * Sets the velocity of the motor appropriate for scoring.
     * 
     * @return A command.
     */
    public Command setVelocity() {
        // Calculates the horizontal distance between the robot and the goal.
        Measure<Distance> hDistance = Meters
                .of(Positioning.robot.getTranslation().getDistance(TARGET.toTranslation2d()));

        // Calculates the vertical distance between the robot and the goal.
        Measure<Distance> vDistance = SHOOTER_HEIGHT.minus(Meters.of(TARGET.getZ()));

        // Uses the trajectory formula to solve for inital velocity
        final double TARGET_VELOCITY = (G.in(MetersPerSecond) * Math.pow(hDistance.in(Meters), 2))
                * Math.pow(1 / Math.cos(LAUNCH_ANGLE.in(Degrees)), 2)
                / (2 * (Math.tan(LAUNCH_ANGLE.in(Degrees)) * hDistance.in(Meters) - vDistance.in(Meters)))
                * POWER_COEFFICIENT;

        return runOnce(() -> pidController.setSetpoint(TARGET_VELOCITY))
                // Sets the setpoint of the PID controller to the target velocity.
                .andThen(run(() -> motor.setVoltage(calculateVoltage().in(Volts))))
                // Sets the motor voltage to the calculated value(based on velocity setpoint).
                .until(() -> pidController.atSetpoint());
        // Stops the command once the motor is at the velocity setpoint.
    }
}
