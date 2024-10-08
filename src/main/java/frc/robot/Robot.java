// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static frc.robot.positioning.PositioningConstants.AUTO_SCORE_POS_1;
import static frc.robot.positioning.PositioningConstants.AUTO_SCORE_POS_2;
import static frc.robot.positioning.PositioningConstants.BALL_ONE_POSITION;
import static frc.robot.positioning.PositioningConstants.BALL_THREE_POSITION;
import static frc.robot.positioning.PositioningConstants.BALL_TWO_POSITION;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.CommandRobot;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.positioning.Positioning;
import frc.robot.shooter.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends CommandRobot {
  private final Intake intake = new Intake();
  private final Elevator elevator = new Elevator();
  private final Shooter shooter = new Shooter();
  private final Drivetrain drivetrain = new Drivetrain();

  private final Field2d field = new Field2d();

  private final CommandXboxController operator = new CommandXboxController(
      Ports.Operator.driverControllerPort);
  private final Commands commands = new Commands(intake, elevator, shooter, drivetrain, operator);

  /** This function is called every 20 ms, no matter the mode. */
  @Override
  public void robotPeriodic() {
    // Updates the command sceduler
    CommandScheduler.getInstance().run();

    // Updates the robot visualizer
    field.setRobotPose(Positioning.robot);
  }

  @Override
  public void autonomousInit() {
    // Runs the shoot command whenever the beambreak is triggered
    commands.BeamBreak().onTrue(commands.shoot());
    CommandScheduler.getInstance()
        .schedule(commands.shoot()
            .andThen(commands.score(BALL_ONE_POSITION)
                .andThen(commands.score(BALL_TWO_POSITION))
                .andThen(commands.score(BALL_THREE_POSITION)))
            .andThen(run(() -> {
              commands.moveTo(AUTO_SCORE_POS_1).andThen(commands.moveTo(AUTO_SCORE_POS_2));
            })));
  }

  @Override
  public void teleopInit() {
    // Cancels all autonomous commands at the beggining of telop
    CommandScheduler.getInstance().cancelAll();

    // Configures the controller bindings
    commands.configureButtonBindings();
  }

  @Override
  public void simulationInit() {
    // Adds field visualizer to dashboard
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void simulationPeriodic() {

  }
}