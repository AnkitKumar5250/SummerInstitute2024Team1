package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.Shooter.*;
import static frc.robot.shooter.ShooterConstants.*;
import static frc.robot.Constants.FieldConstants.*;

// The shooter is to be manually tuned once the robot is all set and working.

// Because of physics(and according to team 254's tech binder) this tuning -
// process only really has to be done for one specific distance, as all of -
// the other distances can be calculated based on this one. 

// Drivetrain needs to add encoders in order to determine the angle at -
// which to shoot(since we need our position relative to the goal)
// Also some weird math is required in order to make sure that we get it -
// into the square. Therefore we just have to aim for the orgin

public class Shooter extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder(); // CHARLIE WHY DID YOU USE AN ABSOLUTE ENCODER YOU
                                                                // ABSOLUTE BUFFOON OF A MAN - michaela x. ??????
                                                                // (wasn't meant in all caps)
    private final PIDController pid = new PIDController(kP, kI, kD);

    /**
     * calculates the amount of power neccesary to score the cell into the bank
     * 
     * @param x the current X position of the robot on the field (requires
     *                 drivetrain input)
     * @param y the current Y position of the robot on the field (requires
     *                 drivetrain input)
     * @return the amount of power neccesary to score the cell into the bank
     */
    public double calcVelocity(double x, double y) {
        double velocity = Math.sqrt((G * Math.pow(x,2) * Math.pow(1/Math.cos(LAUNCH_ANGLE),2))/(2*(Math.tan(LAUNCH_ANGLE)*x-y)));
        return velocity;
    }

    /**
     * condtruocotr
     */
    public Shooter() {

    }

    /**
     * turns off the motor
     */
    public Command turnOff() {
        return run(() -> motor.stopMotor());
    }

    /**
     * returns the speed of the motor in rotations per second
     * 
     * @return speed of the motor in rotations per second
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * sets the speed of the motor using PID
     * 
     * @param velocity the target speed of the motor
     */
    public Command setVelocity(double velocity) {
        return run(() -> motor.setVoltage(pid.calculate(getVelocity(), velocity)));
    }

    /**
     * Updates the speed of the motor based on the position of the robot
     * 
     * @param x X position of robot on the field
     * @param y Y position of robot on the field
     * @return a command that updates the speed of the motor based on the position
     *         of the robot
     */
    public double updateVelocity(double x, double y) {
        double voltage = pid.calculate(getVelocity(), calcVelocity(x, y));
        motor.setVoltage(voltage);
        return voltage;
    }

}
