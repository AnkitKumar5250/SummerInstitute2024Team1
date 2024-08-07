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

  

  private static final CommandXboxController operator = new CommandXboxController(
      Ports.OperatorConstants.driverControllerPort);
  private static final Commands commands = new Commands(intake, elevator, shooter, drivetrain, operator);

  /**
   * This function is called every 20 ms, no matter the mode.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  // //idc
  // @Override
  // public void autonomousInit() {}

  // //idc
  // @Override
  // public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Cancels all autonomous commands at the beggining of telop
    CommandScheduler.getInstance().cancelAll();
    commands.configureButtonBindings();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (elevator.getBeamBreak()) {
      commands.shoot();
    }
  }

}