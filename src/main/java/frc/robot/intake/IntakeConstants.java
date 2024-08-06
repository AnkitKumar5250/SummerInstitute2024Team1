package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class IntakeConstants {
    // PID constants
    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;

    // Angles at which the intake is lowered/raised
    public static final Measure<Angle> EXTENDED_ANGLE = Degrees.of(90);
    public static final Measure<Angle> RETRACTED_ANGLE = Degrees.of(0);

    public static final Measure<Velocity<Distance>> TARGET_VELOCITY =  MetersPerSecond.of(0);
    
   
}
