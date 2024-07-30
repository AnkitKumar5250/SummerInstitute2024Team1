package frc.robot.drivetrain;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FieldConstants.BANK_X;
import static frc.robot.drivetrain.DrivetrainConstants.RightEncoderSourceA;
import static frc.robot.drivetrain.DrivetrainConstants.RightEncoderSourceB;
import static frc.robot.drivetrain.DrivetrainConstants.TURNING_RADIUS;
import static frc.robot.drivetrain.DrivetrainConstants.leftEncoderSourceA;
import static frc.robot.drivetrain.DrivetrainConstants.leftEncoderSourceB;
import static frc.robot.drivetrain.DrivetrainConstants.moveD;
import static frc.robot.drivetrain.DrivetrainConstants.moveI;
import static frc.robot.drivetrain.DrivetrainConstants.moveP;

public class Drivetrain extends SubsystemBase {
    // instantiates motors
    private final CANSparkMax leftLeader = new CANSparkMax(0, kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(1, kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(2, kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(3, kBrushless);
    private final DifferentialDrive DiffDrive = new DifferentialDrive(leftLeader, rightLeader);

    // instantiates encoders
    private final Encoder leftEncoder = new Encoder(leftEncoderSourceA, leftEncoderSourceB);
    private final Encoder rightEncoder = new Encoder(RightEncoderSourceA, RightEncoderSourceB);

    // instantiates PID controllers
    private final PIDController pidControllerRotation = new PIDController(1, 0, 1);
    private final PIDController pidControllerTranslation = new PIDController(moveP, moveI, moveD);

    /**
     * Constructor.
     */
    public Drivetrain() {
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        rightLeader.burnFlash();
        rightFollower.burnFlash();
        leftLeader.burnFlash();
        leftFollower.burnFlash();
    }

    /**
     * Updates voltage based on driver input.
     * 
     * @param ySpeed ...
     * @param zSpeed ...
     */
    public void arcadeDrive(double ySpeed, double zSpeed) {
        DiffDrive.arcadeDrive(ySpeed, zSpeed, true);
    }

    public void drive(double leftSpeed, double rightSpeed) {
        leftLeader.set(Math.copySign(Math.pow(leftSpeed, 2), leftSpeed));
        rightLeader.set(Math.copySign(Math.pow(rightSpeed, 2), -rightSpeed));
    }

    /**
     * Drives based on driver input.
     * 
     * @param leftSpeed  Y-axis of left joystick.
     * @param rightSpeed X-axis of right joystick.
     * @return A command for driving based on driver input.
     */
    public Command driveCommand(double leftSpeed, double rightSpeed) {
        return Commands.run(() -> arcadeDrive(leftSpeed, rightSpeed));
    }

    /**
     * updates voltage based on PID in order to drive a certain distance.
     * 
     * @return voltage of the motors.
     */
    public double driveDistance(double meters) {
        double voltage = 0;
        double encoderValue = leftEncoder.get() + rightEncoder.get() / 2;
        voltage = pidControllerTranslation.calculate(encoderValue, meters);
        drive(Math.sqrt(voltage), Math.sqrt(voltage));
        return voltage;
    }

    /**
     * Drives a certain distance.
     * 
     * @param meters : distance to drive.
     * @return A command that drives a certain distance.
     */
    public Command driveDistanceCommand(double meters) {
        return Commands.run(() -> driveDistance(meters));
    }

    /**
     * updates voltage based on PID in order to fufill rotation command.
     * 
     * @param x : x position of robot
     * @param y : y position of robot
     */
    public double updateDirection(double degrees) {
        double distance = degrees * TURNING_RADIUS * 2 * Math.PI / 360;

        double encoderValue = leftEncoder.get() + rightEncoder.get() / 2;
        double voltage = pidControllerRotation.calculate(encoderValue / 2, distance);

        leftLeader.setVoltage(-voltage);
        rightLeader.setVoltage(voltage);

        return voltage;
    }
}