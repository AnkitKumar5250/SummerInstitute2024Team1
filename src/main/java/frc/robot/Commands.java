package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.positioning.Positioning;
import frc.robot.shooter.Shooter;

public class Commands {
    private Intake intake = new Intake();
    private Elevator elevator = new Elevator();
    private Shooter shooter = new Shooter();
    private Drivetrain drivetrain = new Drivetrain();

    public Commands(Intake intake, Elevator elevator, Shooter shooter, Drivetrain drivetrain) {
        this.intake = intake;
        this.elevator = elevator;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
    }


    /**
     * Configures the button bindings.
     * @param operator : Xbox controller.
     */
    public void configureButtonBindings(CommandXboxController operator) {
        operator.a().whileTrue(IntakeCommand());
        operator.b().onTrue(ShootCommand());
    }

    /**
     * Command to prepare the intake.
     */
    public Command IntakeCommand() {
        return intake.extend()
                .alongWith(intake.startIntake())
                .alongWith(elevator.elevatorBrake())
                .finallyDo(() -> intake.retract());
    }

    /**
     * Command to prepare the shooter.
     */
    public Command ShootCommand() {
        return drivetrain
                .rotateTowardsBank()
                .alongWith(shooter.setVelocity())
                .finallyDo(() -> drivetrain.rotateDegrees(Positioning.calcAngleTowardsBank(), true));
    }

    /**
     * Moves robot to a certain location.
     * 
     * @param x : x coordinate of target location.
     * @param y : y coordinate of target location.
     * @return A command.
     */
    public Command MoveCommand(Measure<Distance> x, Measure<Distance> y) {
        Measure<Distance> distance = Meters.of(Math.hypot(x.in(Meters), y.in(Meters)));
        return drivetrain.rotateTowardsPosition(x, y).finallyDo(() -> drivetrain.driveDistance(distance));
    }

    /**
     * Turns the robot to a certain orientation.
     * 
     * @param angle : angle to rotate to.
     * @return A command.
     */
    private Command RotateCommand(Measure<Angle> angle) {
        return drivetrain.rotateToAngle(angle);
    }
    /**
     * Moves the robot to a certain position on the field.
     * 
     * @param translation : translation representing target location.
     */
    public Command MoveTranslationCommand(Translation2d translation) {
        return MoveCommand(Meters.of(translation.getX()), Meters.of(translation.getY()));
    }

    public Command DriveCommand(Measure<Distance> distance) {
        return drivetrain.driveDistance(distance);
    }

     /**
     * Turns the robot to a certain orientation.
     * 
     * @param angle : angle to rotate to.
     */
    public void RotateTo(Measure<Angle> angle) {
        RotateCommand(angle);
    }
}
