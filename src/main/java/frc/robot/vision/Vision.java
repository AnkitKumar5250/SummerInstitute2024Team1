package frc.robot.vision;

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
public class Vision {
    public static Pose2d Robot = new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(0)));

    /**
     * Calculates the relative angle between the robot's current position and the
     * bank.
     * 
     * @param x : current x position of the robot.
     * @param y : current y position of the robot.
     * @return The angle.
     */
    public static Measure<Angle> calcAngleTowardsBank() {
        return Degrees.of(Math.atan(FieldConstants.BANK_POSITION.getX() - Robot.getX() / 156 - Robot.getY()));
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
        return Degrees.of(Math.atan(x.in(Meters) - Robot.getX() / y.in(Meters) - Robot.getY()));
    }
}
