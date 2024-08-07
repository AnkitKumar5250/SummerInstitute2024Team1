package frc.robot.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Ports.Intake.pivotPort;
import static frc.robot.Ports.Intake.rollerPort;
import static frc.robot.intake.IntakeConstants.*;
import static frc.robot.intake.IntakeConstants.RETRACTED_ANGLE;
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

        /**
         * Constructor
         */
        public Intake() {
                pidControllerPivot.setTolerance(PivotPID.ANGLE_TOLERANCE.in(Degrees));
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
                return runOnce(() -> roller.set(.7))
                .andThen(Commands.idle(this))
                .finallyDo(() -> roller.set(0));
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
