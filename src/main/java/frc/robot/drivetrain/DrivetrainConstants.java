package frc.robot.drivetrain;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.units.Voltage;

/** Constants for the Drivetrain Subsystem. */
public class DrivetrainConstants {
    /** Constants for PID */
    public static final class PID {
        public static final double P = 0; // proportional
        public static final double I = 0; // integral
        public static final double D = 0; // derivative
    }

    /** Constants for FFD */
    public static final class FFD {
        public static final double S = 0; // static gain
        public static final double V = 0; // velocity gain
        public static final double A = 0; // acceleration gain
    }

    // Track width
    public static final Measure<Distance> TRACK_WIDTH = Meters.of(0.7112);

    // Distance per rotation
    public static final double ENCODER_CONVERSION_RATE = 0;

    // Maximum and minimum voltage values of the motor
    public static final Measure<Voltage> MAXIMUM_VOLTAGE = Volts.of(0);
    public static final Measure<Voltage> MINIMUM_VOLTAGE = Volts.of(0);

    // Gearing Reduction
    public static final double GEARING_RATIO = 7.29;

    // Robot mass
    public static final double ROBOT_MASS = 60;

    // Moment of inertia
    public static final double MOMENT_OF_INHERTIA = 7.5;

    // Wheel radius
    public static final Measure<Distance> WHEEL_RADIUS = Inches.of(3);

    // Standart Measurement Deviation
    public static final Vector<N7> STANDART_DEVIATION = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
}