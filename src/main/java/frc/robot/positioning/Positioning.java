package frc.robot.positioning;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.drivetrain.DrivetrainConstants.TRACK_WIDTH;
import static frc.robot.positioning.PositioningConstants.TARGET;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

/**
 * This is utility class designed to aid in accurately positioning the robot.
 */
public class Positioning {
    // Instantiates robot pose
    public static final Pose2d robot = new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(0)));

    /**
     * Calculates the relative angle between the robot's current position and the
     * bank.
     * 
     * @return The angle.
     */
    public static Measure<Angle> calcAngleTowardsBank() {
        // Calculates angle towards the position of the target on the bank.
        return calcAngleTowardsPosition(TARGET.toTranslation2d());
    }

    /**
     * Calculates the relative angle between the robot's current position and
     * another coordinate
     * 
     * @param position
     *            : refrence position.
     * @return The angle.
     */
    public static Measure<Angle> calcAngleTowardsPosition(Translation2d position) {
        // Find the x and y differences and takes the arc tangent of them to find the
        // angle. Then is subtracts the robot's current angle in order to get the
        // relative angle.
        return Degrees.of(Math.atan(Math.abs(position.getX() - robot.getX() / position.getY() - robot.getY())));
    }

    /**
     * Returns the orientation of the robot.
     * 
     * @return Orientation of the robot(0 degrees is facing endzone).
     */
    public static Measure<Angle> getOrientation() {
        // Calculates the orientation of the robot by taking the modulus of the total
        // amount of degrees the robot has traveled(in order to cut out all full
        // rotations)
        return Degrees.of(robot.getRotation().getDegrees() % 360);
    }

    /**
     * Updates the robot pose based on drivetrain input.
     * 
     * @param encoderDisplacement
     *            : Displacement of the encoders relative to the last call of this
     *            function.
     * @param isRotating
     *            : If the robot is rotating or not.
     */
    public static void updateRobotPosition(Measure<Distance> encoderDisplacement, boolean isRotating) {
        // Instantiates robot transform components
        Rotation2d rotation = new Rotation2d();
        Translation2d translation = new Translation2d();

        if (isRotating) {
            // If the robot is rotating, apply encoder displacement to calculate angle
            // rotated
            rotation = new Rotation2d(Degrees.of(encoderDisplacement.in(Meters) / (TRACK_WIDTH.in(Meters) * Math.PI)));
        } else {
            // if the robot is driving, apply encoder displacement to calculate distance
            // traveled(accounting for anglular momentum).
            translation = new Translation2d(robot.getRotation().getCos() * encoderDisplacement.in(Meters),
                    robot.getRotation().getSin() * encoderDisplacement.in(Meters));
        }

        // Applies transform to robot position.
        Positioning.robot.transformBy(new Transform2d(translation, rotation));
    }
}
