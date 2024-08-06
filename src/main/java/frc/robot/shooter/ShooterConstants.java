package frc.robot.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;

public final class ShooterConstants {
    public static final double POWER_COEFFICIENT = 2;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final Measure<Time> SHOOT_TIME = Seconds.of(1);

    public static final Measure<Angle> LAUNCH_ANGLE = Degrees.of(60);

    public static final Measure<Distance> SHOOTER_HEIGHT = Meters.of(0); // 
}