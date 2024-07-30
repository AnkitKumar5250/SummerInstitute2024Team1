package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

public class ScoreCommand extends Command {
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private Elevator elevator;

    public ScoreCommand(Drivetrain drive, Shooter shoot, Intake take, Elevator elev) {
        drivetrain = drive;
        shooter = shoot;
        intake = take;
        elevator = elev;

        addRequirements(drivetrain,elevator,shooter,intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
       return false;
    }
}
