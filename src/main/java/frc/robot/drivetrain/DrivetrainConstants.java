package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class DrivetrainConstants {
    // PID constants
    public static final class PID {
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
    }

    // Turning radius of the robot -> width of track/2
    public static final Measure<Distance> TURNING_RADIUS = Meters.of(0);
    public static final double ENCODER_CONVERSION_RATE = 0;
}