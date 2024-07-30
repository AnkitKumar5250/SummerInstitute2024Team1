package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.BANK_X;
import static frc.robot.shooter.ShooterConstants.voltageCommandEndThreshold;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.shooter.Shooter;

public class ShootCommand extends Command {
    public Drivetrain drivetrain;
    public Shooter shooter;
    public Pose2d position;
    public double degrees;

    public double svoltage;
    public double dvoltage;

    /**
     * instantiates the shoot command alongside the subsystems
     * 
     * @param shooterSubsystem    An instance of the shooter subsystem
     * @param drivetrainSubsystem An instance of the drivetrain subsystem
     */
    public ShootCommand(Drivetrain drivetrainSubsystem, Shooter shooterSubsystem, Pose2d pose2d) {
        drivetrain = drivetrainSubsystem;
        addRequirements(drivetrain);

        shooter = shooterSubsystem;
        addRequirements(shooter);

        position = pose2d;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.updateVelocity(position.getX(), position.getY());
        drivetrain.updateDirection(Math.atan(BANK_X.in(Meters) - position.getX() / 156 - position.getY()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (dvoltage < voltageCommandEndThreshold && svoltage < voltageCommandEndThreshold) {
            return true;
        }
        return false;
    }
}
