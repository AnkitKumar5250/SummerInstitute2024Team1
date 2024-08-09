package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** Constants for the Intake Subsystem. */
public class IntakeConstants {
    /** PID constants for roller */
    public static final class PivotPID {
        public static final double P = 0; // proportional
        public static final double I = 0; // integral
        public static final double D = 0; // derivative

        // The maximum allowed error when it comes to the angle of the pivot
        public static final Measure<Angle> ANGLE_TOLERANCE = Degrees.of(0);
    }

    /** FFD constants for pivot */
    public static final class PivotFFD {
        public static final double S = 0; // static gain
        public static final double V = 0; // velocity gain
        public static final double G = 0; // gravitational gain
        public static final double A = 0; // acceleration gain
    }

    // Angles at which the intake is lowered/raised
    public static final Measure<Angle> EXTENDED_ANGLE = Degrees.of(90);
    public static final Measure<Angle> RETRACTED_ANGLE = Degrees.of(0);

    // Target velocity of the roller
    public static final Measure<Voltage> TARGET_VOLTAGE = Volts.of(0);

    // How many volts is equivalent to 1 m/s of output on the pivot motor
    public static final Measure<Per<Voltage, Velocity<Distance>>> VOLTS_TO_VELOCTIY = VoltsPerMeterPerSecond.of(0);

    // Maximum and minimum voltage values of the pivot motor
    public static final Measure<Voltage> MAXIMUM_VOLTAGE = Volts.of(0);
    public static final Measure<Voltage> MINIMUM_VOLTAGE = Volts.of(0);

    // Constants to convert from encoder rotations to meters
    public static final Measure<Distance> ENCODER_CONVERSION_FACTOR = Meters.of(0);

}
