package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class IntakeConstants {
    // PID constants for pivot
    public static final class PivotPID {
        public static final double P = 0; // proportional
        public static final double I = 0; // integral
        public static final double D = 0; // derivative

        // The maximum allowed error when it comes to the angle of the pivot
        public static final Measure<Angle> ANGLE_TOLERANCE = Degrees.of(0);
    }

    // PID constants for roller
    public static final class RollerPID {
        public static final double P = 0; // proportional
        public static final double I = 0; // integral
        public static final double D = 0; // derivative

        // The maximum allowed error when it comes to the velocity of the roller
        public static final Measure<Velocity<Distance>> VELOCITY_TOLERANCE = MetersPerSecond.of(0);
    }
   

    // Angles at which the intake is lowered/raised
    public static final Measure<Angle> EXTENDED_ANGLE = Degrees.of(90);
    public static final Measure<Angle> RETRACTED_ANGLE = Degrees.of(0);

    // Target velocity of the roller
    public static final Measure<Velocity<Distance>> TARGET_VELOCITY =  MetersPerSecond.of(0);
    
   
}
