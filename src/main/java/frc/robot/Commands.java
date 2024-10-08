package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.positioning.Positioning;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterConstants;

public class Commands {
    private final Intake intake;
    private final Elevator elevator;
    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private final CommandXboxController operator;

    public Commands(Intake intake, Elevator elevator, Shooter shooter, Drivetrain drivetrain,
            CommandXboxController operator) {
        this.intake = intake;
        this.elevator = elevator;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.operator = operator;
    }

    /**
     * Configures the button bindings.
     * 
     * @param operator
     *            : Xbox controller.
     */
    public void configureButtonBindings() {
        operator.rightBumper().whileTrue(intake());
        operator.leftBumper().onTrue(shoot());
        drivetrain.setDefaultCommand(drivetrain.drive(operator.getLeftY(), operator.getRightX()));
    }

    /**
     * Triggers a command if the beambreak gets activated.
     * 
     * @return A trigger.
     */
    public Trigger BeamBreak() {
        return new Trigger(() -> elevator.getBeamBreak());
    }

    /**
     * Command to prepare the intake.
     * 
     * @return A command.
     */
    public Command intake() {
        return intake.extend().alongWith(intake.startRoller())
                .finallyDo(() -> intake.stopRoller().alongWith(intake.retract()));
    }

    /**
     * Command to launch the ball.
     * 
     * @return A command.
     */
    public Command shoot() {
        return drivetrain
                .rotateTowardsBank().alongWith(elevator.start())
                .alongWith(shooter.setVelocity()).withTimeout(ShooterConstants.SHOOT_TIME.in(Seconds))
                .finallyDo(() -> drivetrain.rotateDegrees(Positioning.calcAngleTowardsBank().negate())
                        .alongWith(shooter.turnOff()).alongWith(elevator.stop()));
    }

    /**
     * Drives a certain distance.
     * 
     * @param distance
     *            : distance to drive.
     * @return A command.
     */
    public Command drive(Measure<Distance> distance) {
        return drivetrain.drive(distance);
    }

    /**
     * Moves robot to a certain location.
     * 
     * @param x
     *            : x coordinate of target location.
     * @param y
     *            : y coordinate of target location.
     * @return A command.
     */
    public Command driveTo(Measure<Distance> x, Measure<Distance> y) {
        return drivetrain.rotateTowardsPosition(new Translation2d(x, y))
                .andThen(() -> drivetrain.drive(Meters.of(Math.hypot(x.in(Meters), y.in(Meters)))));
    }

    /**
     * Moves the robot to a certain position on the field.
     * 
     * @param translation
     *            : translation representing target location.
     * @return A command.
     */
    public Command driveTo(Translation2d translation) {
        return driveTo(Meters.of(translation.getX()), Meters.of(translation.getY()));
    }

    /**
     * Turns the robot a certain amount of degrees.
     * 
     * @param angle
     *            : angle to rotate by.
     * @return A command.
     */
    public Command rotate(Measure<Angle> angle) {
        return drivetrain.rotateDegrees(angle);
    }

    /**
     * Turns the robot to a certain orientation.
     * 
     * @param angle
     *            : angle to rotate to.
     * @return A command.
     */
    public Command rotateTo(Measure<Angle> angle) {
        return drivetrain.rotateToAngle(angle);
    }

    /**
     * Turns the robot to a certain orientation.
     * 
     * @param rotation
     *            : rotation to rotate to.
     * @return A command.
     */
    public Command rotateTo(Rotation2d rotation) {
        return drivetrain.rotateToAngle(Degrees.of(rotation.getDegrees() % 360));
    }

    /**
     * Moves the robot to a certain position and rotation.
     * 
     * @param position
     *            : pose 2d representing the position.
     * @return A command.
     */
    public Command moveTo(Pose2d position) {
        return driveTo(position.getTranslation()).andThen(rotateTo(position.getRotation()));
    }

    /**
     * Scores a ball at a certain position
     * 
     * @param ballPosition
     *            : position of the ball
     * @return A command.
     */
    public Command score(Translation2d ballPosition) {
        return intake().andThen(driveTo(ballPosition).finallyDo(() -> shoot()).alongWith(runOnce(() -> intake().cancel())));
    }

}
