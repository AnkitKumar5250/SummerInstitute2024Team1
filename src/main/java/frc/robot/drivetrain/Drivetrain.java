package frc.robot.drivetrain;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Suppliers.MinVoltSupplier;

import static frc.robot.drivetrain.DrivetrainConstants.*;

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

    /**
     * Drives based on driver input.
     * 
     * @param leftSpeed  Y-axis of left joystick.
     * @param rightSpeed X-axis of right joystick.
     * @return A command for driving based on driver input.
     */
    public void tankdrive(double leftSpeed, double rightSpeed) {
        DiffDrive.tankDrive(leftSpeed, -rightSpeed);
    }

    public double calcAngle(double x, double y) {
        return FieldConstants.BANK_X.in(Meters) - x / 156 - y;
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

        if (voltage > 1) {
            voltage = 1;
        }

        leftLeader.set(voltage);
        rightLeader.set(voltage);

        return voltage;
    }

    /**
     * updates voltage based on PID in order to fufill rotation command.
     * 
     * @param degrees : degrees to rotate
     */
    public double rotateDegrees(double degrees) {
        double distance = degrees * TURNING_RADIUS * 2 * Math.PI / 360;

        double encoderValue = leftEncoder.get() + rightEncoder.get() / 2;
        double voltage = pidControllerRotation.calculate(encoderValue / 2, distance);

        if (voltage > 1) {
            voltage = 1;
        }

        leftLeader.setVoltage(-voltage);
        rightLeader.setVoltage(voltage);

        return voltage;
    }

     /**
     * updates voltage based on PID in order to fufill rotation command.
     * 
     * @param x : x position of robot
     * @param y : y position of robot
     */
    public double rotateDegrees(double x, double y) {
        double distance = (FieldConstants.BANK_X.in(Meters) - x / 156 - y) * TURNING_RADIUS * 2 * Math.PI / 360;

        double encoderValue = leftEncoder.get() + rightEncoder.get() / 2;
        double voltage = pidControllerRotation.calculate(encoderValue / 2, distance);
        
        if (Math.abs(voltage) > 1) {
            voltage = Math.copySign(1, voltage);
        }

        leftLeader.setVoltage(-voltage);
        rightLeader.setVoltage(voltage);

        return voltage;
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
     * Drives a certain distance.
     *
     * @param meters : distance to drive.
     * @return A command that drives a certain distance.
     */
    public Command driveDistanceCommand(double meters) {
        return Commands.run(() -> driveDistance(meters)).until(new MinVoltSupplier(leftLeader.getBusVoltage()));
    }

    /**
     * Drives a certain distance.
     *
     * @param meters : distance to drive.
     * @return A command that drives a certain distance.
     */
    public Command rotateDegreesCommand(double degrees) {
        return Commands.run(() -> rotateDegrees(degrees)).until(new MinVoltSupplier(leftLeader.getBusVoltage()));
    }

     /**
     * Drives a certain distance.
     *
     * @param meters : distance to drive.
     * @return A command that drives a certain distance.
     */
    public Command rotateDegreesCommand(double x, double y) {
        return Commands.run(() -> rotateDegrees(x,y)).until(new MinVoltSupplier(leftLeader.getBusVoltage()));
    }

}