package frc.robot.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Ports.Intake.pivotPort;
import static frc.robot.Ports.Intake.rollerPort;
import static frc.robot.intake.IntakeConstants.*;
import static frc.robot.intake.IntakeConstants.RETRACTED_ANGLE;
import static frc.robot.intake.IntakeConstants.TARGET_VELOCITY;
import static frc.robot.intake.IntakeConstants.EXTENDED_ANGLE;

public class Intake extends SubsystemBase {
        // Instantiates motors
        private final CANSparkMax roller = new CANSparkMax(rollerPort, MotorType.kBrushless);
        private final CANSparkMax pivot = new CANSparkMax(pivotPort, MotorType.kBrushless);

        // Instantiates encoders
        private final AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();
        private final RelativeEncoder rollerEncoder = roller.getEncoder();

        // Instantiates controllers
        private final PIDController pidControllerPivot = new PIDController(PivotPID.P, PivotPID.I, PivotPID.D);
        private final PIDController pidControllerRoller = new PIDController(RollerPID.P, RollerPID.I, RollerPID.D);

        /**
         * Constructor
         */
        public Intake() {
                pidControllerPivot.setTolerance(PivotPID.ANGLE_TOLERANCE.in(Degrees));
                pidControllerRoller.setTolerance(RollerPID.VELOCITY_TOLERANCE.in(MetersPerSecond));
        }

        /**
         * Extends the intake.
         *
         * @return A command.
         */
        public Command extend() {
                return runOnce(() -> pidControllerPivot.setSetpoint(EXTENDED_ANGLE.in(Degrees))).andThen(run(
                                () -> pivot.setVoltage(pidControllerPivot.calculate(pivotEncoder.getPosition() * 360)))
                                .until(() -> pidControllerPivot.atSetpoint()));
        }

        /**
         * Retracts the intake.
         *
         * @return A command.
         */
        public Command retract() {
                return runOnce(() -> pidControllerPivot.setSetpoint(RETRACTED_ANGLE.in(Degrees)))
                                .andThen(run(() -> pivot.setVoltage(
                                                pidControllerPivot.calculate(pivotEncoder.getPosition() * 360)))
                                                .until(() -> pidControllerPivot.atSetpoint()));

        }

        /**
         * Starts the rollers.
         *
         * @return A command.
         */
        public Command start() {
                return runOnce(() -> pidControllerRoller.setSetpoint(TARGET_VELOCITY.in(MetersPerSecond))).andThen(run(
                                () -> roller.set(
                                                pidControllerRoller.calculate(rollerEncoder.getVelocity())))
                                .until(() -> pidControllerRoller.atSetpoint()));
        }

        /**
         * Stops the rollers.
         *
         * @return A command.
         */
        public Command stop() {
                return runOnce(() -> roller.stopMotor());
        }

}
