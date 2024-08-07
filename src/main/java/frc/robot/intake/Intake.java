package frc.robot.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
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

        // Instantiates controllers
        private final PIDController pidControllerPivot = new PIDController(PivotPID.P, PivotPID.I, PivotPID.D);
        private final ArmFeedforward armFeedforward = new ArmFeedforward(PivotFFD.S, PivotFFD.G, PivotFFD.V,
                        PivotFFD.A);

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
                return runOnce(() -> pidControllerPivot.setSetpoint(EXTENDED_ANGLE.in(Degrees)))
                .andThen(run(() -> pivot.setVoltage(
                                pidControllerPivot.calculate(pivotEncoder.getPosition() * 360)
                                                + armFeedforward.calculate(EXTENDED_ANGLE.in(Radians),
                                                                pivotEncoder.getVelocity())))
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
                                                pidControllerPivot.calculate(pivotEncoder.getPosition() * 360)
                                                                + armFeedforward.calculate(RETRACTED_ANGLE.in(Radians),
                                                                                pivotEncoder.getVelocity())))
                                                .until(() -> pidControllerPivot.atSetpoint()));

        }

        /**
         * Starts the rollers.
         *
         * @return A command.
         */
        public Command startRoller() {
                return runOnce(() -> roller.setVoltage(TARGET_VOLTAGE.in(Volts)));
        }

        /**
         * Stops the rollers.
         *
         * @return A command.
         */
        public Command stopRoller() {
                return runOnce(() -> roller.stopMotor());
        }

}
