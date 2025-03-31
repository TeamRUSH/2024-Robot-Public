package net.teamrush27.frc2024.subsystems.leds;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import net.teamrush27.frc2024.subsystems.leds.LedIO.LEDOutputs;

public enum LedWantedState {
    DISABLED(new SingleFadeAnimation(30, 65, 124, 0, 0.5, LedConfig.NUM_LED)),
    IDLE(new StrobeAnimation(30, 65, 124)),
    LOW_BATTERY(255, 0, 0),
    GOOD_BATTERY(new ColorFlowAnimation(0, 255, 0)),
    RAINBOW(new RainbowAnimation(0.2, 0.2, LedConfig.NUM_LED)),
    EXHAUST(new StrobeAnimation(255, 0, 0, 0, .3, LedConfig.NUM_LED)),
    INTAKE_NOTE(255, 0, 0),
//    INTAKE_NOTE(new FireAnimation()),
    HAS_NOTE(255, 180, 0),
    LOADED_STROBE(new StrobeAnimation(0, 255, 0, 0, 0.3, LedConfig.NUM_LED)),
    LOADED_SOLID(0, 255, 0),
    ALIGNED(new LarsonAnimation(255, 255, 255, 0, 1, LedConfig.NUM_LED, BounceMode.Front, LedConfig.NUM_LED, 0)),
    ALIGNED_AT_SPEED(255, 255, 255),
    FIRE(new StrobeAnimation(255, 255, 255, 0, 0.5, LedConfig.NUM_LED)),
    FAULT(new StrobeAnimation(255, 0, 0, 0, 0.1, LedConfig.NUM_LED)),
    READY_TO_FIRE(new StrobeAnimation(255,255,255));

    private Animation animation;

    private int r = -1;
    private int g = -1;
    private int b = -1;

    private LedWantedState(Animation animation) {
        this.animation = animation;
    }

    private LedWantedState(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public Animation getAnimation() {
        return animation;
    }


    public boolean isAnimation(){
        return r == -1;
    }

    public void setOutputs(LEDOutputs ledOutputs) {
        if(!isAnimation()) {
            ledOutputs.r = this.r;
            ledOutputs.g = this.g;
            ledOutputs.b = this.b;
        }
        if (isAnimation()) {
            ledOutputs.animation = this.animation;
        }
    }
}