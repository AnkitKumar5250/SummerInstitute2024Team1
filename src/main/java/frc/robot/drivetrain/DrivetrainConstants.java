package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** Constants for the Drivetrain Subsystem. */
public class DrivetrainConstants {
    /** Constants for PID */
    public static final class PID {
        public static final double P = 0; // proportional
        public static final double I = 0; // integral
        public static final double D = 0; // derivative
    }

    // Turning radius of the robot -> width of track/2
    public static final Measure<Distance> TURNING_RADIUS = Meters.of(0);
    public static final double ENCODER_CONVERSION_RATE = 0;

    // Maximum and minimum voltage values of the motor
    public static final Measure<Voltage> MAXIMUM_VOLTAGE = Volts.of(0);
    public static final Measure<Voltage> MINIMUM_VOLTAGE = Volts.of(0);

    // How many volts is equivalent to 1 m/s of output
    public static final Measure<Per<Voltage, Velocity<Distance>>> VOLTS_TO_VELOCTIY = VoltsPerMeterPerSecond.of(0);
}