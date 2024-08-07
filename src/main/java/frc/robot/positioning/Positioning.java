package frc.robot.positioning;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.drivetrain.DrivetrainConstants.TURNING_RADIUS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.robot.Constants.FieldConstants;

/**
 * This is utility class designed to aid in accurately positioning the robot.
 */
public class Positioning {
    // Instantiates robot pose
    public static Pose2d robot = new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(0)));

    /**
     * Calculates the relative angle between the robot's current position and the
     * bank.
     * 
     * @param x
     *            : current x position of the robot.
     * @param y
     *            : current y position of the robot.
     * @return The angle.
     */
    public static Measure<Angle> calcAngleTowardsBank() {
        return Degrees.of(Math.atan(FieldConstants.BANK_POSITION.getX() - robot.getX() / 156 - robot.getY()))
                .minus(Positioning.getOrientation());
    }

    /**
     * Calculates the relative angle between the robot's current position and
     * another coordinate
     * 
     * @param x
     *            : current x position of the robot.
     * @param y
     *            : current x position of the robot.
     * @param x
     *            : refrence point X.
     * @param y
     *            : refrence point Y.
     * @return The angle.
     */
    public static Measure<Angle> calcAngleTowardsPosition(Measure<Distance> x, Measure<Distance> y) {
        return Degrees.of(Math.atan(x.in(Meters) - robot.getX() / y.in(Meters) - robot.getY()))
                .minus(Positioning.getOrientation());
    }

    public static Measure<Angle> getOrientation() {
        return Degrees.of(robot.getRotation().getDegrees() % 360);
    }

    /**
     * Updates the robot pose based on drivetrain input.
     * 
     * @param encoderValue
     *            : Measure of the encoders.
     * @param isRotating
     *            : If the robot is rotating or not.
     */
    public static void updateRobotPosition(double encoderValue, boolean isRotating) {
        Rotation2d rotation = new Rotation2d();
        Translation2d translation = new Translation2d();
        Transform2d transform;
        if (isRotating) {
            rotation = new Rotation2d(encoderValue / (TURNING_RADIUS.in(Meters) * 2 * Math.PI / 360));
        } else {
            double xComp = robot.getRotation().getCos() * encoderValue;
            double yComp = robot.getRotation().getSin() * encoderValue;
            translation = new Translation2d(xComp, yComp);
        }

        transform = new Transform2d(translation, rotation);
        Positioning.robot.transformBy(transform);
    }
}
