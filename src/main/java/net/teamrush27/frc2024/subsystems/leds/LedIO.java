package net.teamrush27.frc2024.subsystems.leds;


import com.ctre.phoenix.led.Animation;

public interface LedIO {

    public static class LEDOutputs {
        Animation animation;
        int r;
        int g;
        int b;
    }

}
