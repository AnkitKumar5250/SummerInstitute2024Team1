// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.CommandRobot;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.*;

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
  private static final Intake intake = new Intake();
  private static final Elevator elevator = new Elevator();
  private static final Shooter shooter = new Shooter();
  private static final Drivetrain drivetrain = new Drivetrain();

  private static final Commands Actions = new Commands(intake, elevator, shooter, drivetrain);

  private static final CommandXboxController driver = new CommandXboxController(
      Ports.OperatorConstants.driverControllerPort);
  private static final CommandXboxController operator = new CommandXboxController(
      Ports.OperatorConstants.OperatorControllerPort);

  /**
   * This function is called every 20 ms, no matter the mode.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    Actions.runIntake();
    Actions.moveTo(BALL_ONE_POSITION);
    Actions.shoot();
    Actions.moveTo(BALL_TWO_POSITION);
    Actions.shoot();
    Actions.moveTo(BALL_THREE_POSITION);
    Actions.shoot();

    // This should cause the robot to shoot after picking up each ball, then move to the autoscore positions

    Actions.moveTo(AUTO_SCORE_POS_1);
    Actions.rotate(Degrees.of(90));
    Actions.drive(Meters.of(12));
    Actions.shoot();
    Actions.moveTo(AUTO_SCORE_POS_2);
    Actions.shoot();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Runs shoot command when beambrake is triggered
    if (elevator.getBeamBreak()) {
      Actions.shoot();
    }

  }

  @Override
  public void teleopInit() {
    // Cancels all autonomous commands at the beggining of telop
    CommandScheduler.getInstance().cancelAll();
    Actions.configureButtonBindings(operator);
    drivetrain.setDefaultCommand(drivetrain.drive(driver.getLeftY(), driver.getRightY()));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (elevator.getBeamBreak()) {
      Actions.shoot();
    }
  }

}