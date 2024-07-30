// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
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
public class Robot extends TimedRobot {
  private static final Intake intake = new Intake();
  private static final Elevator elevator = new Elevator();
  private static final Shooter shooter = new Shooter();
  private static final Drivetrain drivetrain = new Drivetrain();
  private static final CommandXboxController driver = new CommandXboxController(
      Ports.OperatorConstants.driverControllerPort);
  private static final CommandXboxController operator = new CommandXboxController(Ports.OperatorConstants.OperatorControllerPort);


  private static Pose2d position = new Pose2d(0, 0, new Rotation2d());

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically during disabled mode. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (elevator.getBeamBreak()) {
      CommandScheduler.getInstance().schedule(new ShootCommand(drivetrain, shooter, position));
    }
  }

  @Override
  public void teleopInit() {
    operator.a().whileTrue(
        Commands.sequence(
            intake.extend()
                .alongWith(intake.runIntake())
                .alongWith(elevator.elevatorBrake()))
            .finallyDo(() -> intake.retract()));

    operator.b().onTrue(shooter.turnOff());
  }

  /** This function is called periodically during operator control. */
  // Once a is held down the intake will extend and be activated along with the
  // elevator until the beambreak is triggered
  @Override
  public void teleopPeriodic() {
    drivetrain.arcadeDrive(driver.getLeftY(), driver.getRightX());

    if (elevator.getBeamBreak()) {
      CommandScheduler.getInstance().schedule(new ShootCommand(drivetrain, shooter, position).until(operator.x()));
    }
  }

  /** This function is called once each time the robot enters testing mode. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}