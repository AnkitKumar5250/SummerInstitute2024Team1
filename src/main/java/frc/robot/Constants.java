// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVE_CONTROLLER_PORT = 0; //
  }

  public static class FieldConstants {
    // 0,0 is at the center
    // -90,156 is at the top left
    // 90,156 is at the top right
    // -90, -156 is at the bottom left
    // 90, -156 is at the bottom right
    // positive Y is up the field
    // positive X is to the right

    public static final Translation2d BANK_POSITION = new Translation2d(Inches.of(132), Inches.of(90));

    public static final Translation2d BALL_ONE_POSITION = new Translation2d(Inches.of(126), Inches.of(-10));
    public static final Translation2d BALL_TWO_POSITION = new Translation2d(Inches.of(90), Inches.of(-10));
    public static final Translation2d BALL_THREE_POSITION = new Translation2d(Inches.of(54), Inches.of(-10));

    public static final Pose2d AUTO_SCORE_POS_1 = new Pose2d(Inches.of(126), Inches.of(-10),
        new Rotation2d(Degrees.of(90)));
    public static final Pose2d AUTO_SCORE_POS_2 = new Pose2d(Inches.of(90), Inches.of(-10),
        new Rotation2d(Degrees.of(-90)));

    public static final Translation3d TARGET = new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0));

    // dimensions of field

    public static final Measure<Distance> BANK_LENGTH = Inches.of(36);
    public static final Measure<Distance> BANK_WIDTH = Inches.of(48);

    public static final Measure<Distance> FIELD_LENGTH = Inches.of(312);
    public static final Measure<Distance> FIELD_WIDTH = Inches.of(180);

    public static final Measure<Distance> HUMAN_PLAYER_ZONE_LENGTH = Inches.of(24);
    public static final Measure<Distance> HUMAN_PLAYER_ZONE_WIDTH = Inches.of(36);

    public static final Measure<Distance> CELL_BANK_ZONE_LENGTH = Inches.of(87);
    public static final Measure<Distance> CELL_BANK_ZONE_WIDTH = Inches.of(180);

    public static final Measure<Distance> MID_ZONE_LENGTH = Inches.of(68);
    public static final Measure<Distance> MID_ZONE_WIDTH = Inches.of(180);

    public static final Measure<Distance> END_ZONE_LENGTH = Inches.of(155);
    public static final Measure<Distance> END_ZONE_WIDTH = Inches.of(180);

  }

  public static final Measure<Velocity<Distance>> G = MetersPerSecond.of(9.81);
  public static final Measure<Voltage> MINIMUM_VOLTAGE_THRESHHOLD = Volts.of(0.1); //
}
