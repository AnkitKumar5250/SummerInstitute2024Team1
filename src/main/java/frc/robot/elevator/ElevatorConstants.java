package frc.robot.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public class ElevatorConstants {
    // PID constants
    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;

    // Voltage that the elevator runs at
    public static final Measure<Voltage> TARGET_VOLTAGE = Volts.of(.25);
}
