package net.teamrush27.frc2024.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import net.teamrush27.frc2024.Constants;

public class LedConfig {

    public static final int CANDLE_ID = 50;
    public static final int NUM_LED = 500;

    public static CANdle initCANdle() {
        CANdle candle = new CANdle(CANDLE_ID, Constants.CANBUS_NAME);
        candle.configBrightnessScalar(1);
        candle.configLEDType(CANdle.LEDStripType.GRB);
        candle.configLOSBehavior(true);
        candle.configStatusLedState(false);
        return candle;
    }

}