// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private static final Intake intake = new Intake();
  private static final Elevator elevator = new Elevator();
  private static final Shooter shooter = new Shooter();
  private static final Drivetrain drivetrain = new Drivetrain();
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

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (elevator.getBeamBreak()) {
      Shoot();
    }

  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(drivetrain.driveCommand(driver.getLeftY(), driver.getRightY()));

    // Once a is held down the intake will extend and be activated along with the
    // elevator until the beambreak is triggered
    operator.a().whileTrue(IntakeCommand());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (elevator.getBeamBreak()) {
      Shoot();
    }
  }

  /**
   * Command to prepare the intake
   */
  private Command IntakeCommand() {
    return intake.extend()
        .alongWith(intake.startIntake())
        .alongWith(elevator.elevatorBrake())
        .finallyDo(() -> intake.retract());
  }

  /**
   * Command to prepare the shooter
   */
  private Command ShootCommand() {
    return drivetrain
        .rotateTowardsBankCommand()
        .alongWith(shooter.setVelocity())
        .finallyDo(() -> drivetrain.rotateDegreesCommand(Positioning.calcAngleTowardsBank(), true));
  }

  private Command MoveCommand(Measure<Distance> x, Measure<Distance> y) {
    Measure<Distance> distance = Meters.of(Math.hypot(x.in(Meters), y.in(Meters)));
    return drivetrain.rotateTowardsPositionCommand(x, y).finallyDo(() -> drivetrain.driveDistanceCommand(distance));
  }

  private Command RotateCommand(Measure<Angle> angle) {
    return drivetrain.rotateToAngleCommand(angle);
  }



  /**
   * Prepares the intake.
   */
  private void Intake() {
    CommandScheduler.getInstance().schedule(IntakeCommand());
  }

  /**
   * Launches the ball at the goal.
   */
  private void Shoot() {
    CommandScheduler.getInstance().schedule(ShootCommand());
  }

  /**
   * Moves the robot to a certain position on the field.
   * @param translation : translation representing target location.
   */
  private void Move(Translation2d translation) {
    CommandScheduler.getInstance().schedule(MoveCommand(Meters.of(translation.getX()),Meters.of(translation.getY())));
  }

}