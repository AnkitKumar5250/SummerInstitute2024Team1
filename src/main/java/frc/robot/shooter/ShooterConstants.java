package frc.robot.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;

/** Constants for the Shooter Subsystem. */
public final class ShooterConstants {
    // Multiplies power of the shooter
    public static final double POWER_COEFFICIENT = 2;

    /** Constants for PID */
    public static final class PID {
        public static final double kP = 0; // proportional
        public static final double kI = 0; // integral
        public static final double kD = 0; // derivative

        // Maximum accepted error in velocity of the motor
        public static final Measure<Velocity<Distance>> VELOCITY_TOLERANCE = MetersPerSecond.of(0);
    }

    // How long it takes to launch the ball from the start of the command
    public static final Measure<Time> SHOOT_TIME = Seconds.of(1);

    // The angle at which the ball is launched
    public static final Measure<Angle> LAUNCH_ANGLE = Degrees.of(60);

    // The height of the point where the ball is launched
    public static final Measure<Distance> SHOOTER_HEIGHT = Meters.of(0); //
}