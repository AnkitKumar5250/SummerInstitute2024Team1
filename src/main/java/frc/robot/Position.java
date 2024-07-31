package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import frc.robot.Constants.FieldConstants;

/**
 * This is utility class designed to aid in accurately positioning the robot.
 */
public class Position {
    public static Measure<Distance> X = Meters.of(0);
    public static Measure<Distance> Y = Meters.of(0);
    public static Rotation2d angle = new Rotation2d(Degrees.of(0));
    public static Pose2d ROBOT = new Pose2d(X, Y, angle);
    
    /**
     * Calculates the relative angle between the robot's current position and the
     * bank.
     * 
     * @param x : current x position of the robot.
     * @param y : current y position of the robot.
     * @return The angle.
     */
    public static Measure<Angle> calcAngleTowardsBank() {
        return Degrees.of(Math.atan(FieldConstants.BANK_X.in(Meters) - X.in(Meters) / 156 - Y.in(Meters)));
    }

    /**
     * Calculates the relative angle between the robot's current position and
     * another coordinate
     * 
     * @param x : current x position of the robot.
     * @param y : current x position of the robot.
     * @param x : refrence point X.
     * @param y : refrence point Y.
     * @return The angle.
     */
    public static Measure<Angle> calcAngleTowardsPosition(Measure<Distance> x, Measure<Distance> y) {
        return Degrees.of(Math.atan(x.in(Meters) - X.in(Meters) / y.in(Meters) - Y.in(Meters)));
    }
}
