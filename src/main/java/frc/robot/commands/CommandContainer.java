package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.positioning.Positioning;
import frc.robot.shooter.Shooter;

public class CommandContainer {
    private Intake intake = new Intake();
    private Elevator elevator = new Elevator();
    private Shooter shooter = new Shooter();
    private Drivetrain drivetrain = new Drivetrain();

    public CommandContainer(Intake intake, Elevator elevator, Shooter shooter, Drivetrain drivetrain) {
        this.intake = intake;
        this.elevator = elevator;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
    }


    private void configureBindings(CommandXboxController operator) {
        operator.a().whileTrue(IntakeCommand());
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

    /**
     * Moves robot to a certain location.
     * 
     * @param x : x coordinate of target location.
     * @param y : y coordinate of target location.
     * @return A command.
     */
    private Command MoveCommand(Measure<Distance> x, Measure<Distance> y) {
        Measure<Distance> distance = Meters.of(Math.hypot(x.in(Meters), y.in(Meters)));
        return drivetrain.rotateTowardsPositionCommand(x, y).finallyDo(() -> drivetrain.driveDistanceCommand(distance));
    }

    /**
     * Turns the robot to a certain orientation.
     * 
     * @param angle : angle to rotate to.
     * @return A command.
     */
    private Command RotateCommand(Measure<Angle> angle) {
        return drivetrain.rotateToAngleCommand(angle);
    }

    /**
     * Prepares the intake.
     */
    public void Intake() {
        CommandScheduler.getInstance().schedule(IntakeCommand());
    }

    /**
     * Launches the ball at the goal.
     */
    public void Shoot() {
        CommandScheduler.getInstance().schedule(ShootCommand());
    }

    /**
     * Moves the robot to a certain position on the field.
     * 
     * @param translation : translation representing target location.
     */
    public void Move(Translation2d translation) {
        CommandScheduler.getInstance()
                .schedule(MoveCommand(Meters.of(translation.getX()), Meters.of(translation.getY())));
    }

     /**
     * Turns the robot to a certain orientation.
     * 
     * @param angle : angle to rotate to.
     */
    private void RotateTO(Measure<Angle> angle) {
        RotateCommand(angle);
    }
}
