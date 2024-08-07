package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

/** Constants for the Drivetrain Subsystem. */
public class DrivetrainConstants {
    /** Constants for PID */
    public static final class PID {
        public static final double P = 0; // proportional
        public static final double I = 0; // integral
        public static final double D = 0; // derivative
    }

    // Turning radius of the robot
    public static final Measure<Distance> TURNING_RADIUS = Meters.of(0);
}