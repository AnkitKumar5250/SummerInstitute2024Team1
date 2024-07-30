package frc.robot.Suppliers;

import java.util.function.*;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.FieldConstants.*;

public class MinVoltSupplier implements BooleanSupplier{

    private double voltage;

    public MinVoltSupplier(double voltage) {
        this.voltage = voltage;
    }
    @Override
    public boolean getAsBoolean() {
        if (this.voltage < MINIMUM_VOLTAGE_THRESHHOLD.in(Volts)) {
            return true;
        }
        return false;
    }
    
}
