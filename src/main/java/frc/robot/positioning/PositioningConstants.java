package frc.robot.positioning;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Constants for Positioning. */
public class PositioningConstants {
    // (0,0) is at the corner to the right of the cell bank

    // position of the bank
    public static final Translation2d BANK_POSITION = new Translation2d(Inches.of(18), Inches.of(90));

    // position of the three balls lines up at the edge of the endzone
    public static final Translation2d BALL_ONE_POSITION = new Translation2d(Inches.of(47), Inches.of(157));
    public static final Translation2d BALL_TWO_POSITION = new Translation2d(Inches.of(90), Inches.of(157));
    public static final Translation2d BALL_THREE_POSITION = new Translation2d(Inches.of(47), Inches.of(157));

    // Positions on the left and right of the human player zone where the robot will osolate between while picking up balls
    public static final Pose2d AUTO_SCORE_POS_1 = new Pose2d(Inches.of(37), Inches.of(300),
            new Rotation2d(Degrees.of(90)));
    public static final Pose2d AUTO_SCORE_POS_2 = new Pose2d(Inches.of(142), Inches.of(300),
            new Rotation2d(Degrees.of(-90)));

    // 3D coordinate that marks the exact position in which the ball has to make contact with the cell bank
    public static final Translation3d TARGET = new Translation3d(Inches.of(18), Inches.of(90), Inches.of(54));
}
