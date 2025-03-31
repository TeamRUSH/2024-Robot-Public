package net.teamrush27.frc2024.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;

import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import net.teamrush27.frc2024.Robot;
import net.teamrush27.frc2024.subsystems.vision.Vision;

public class Led extends Subsystem {

    @IgnoreLogged
    private static Led instance;

    public static Led getInstance() {
        if (instance == null) {
            instance = new Led();
        }
        return instance;
    }

    private final CANdle candle;
    @IgnoreLogged
    private final Vision vision;

    private final LedIO.LEDOutputs outputs = new LedIO.LEDOutputs();

    @Log
    private LedWantedState wantedState = LedWantedState.DISABLED;

    private Led() {
        candle = LedConfig.initCANdle();
        vision = Vision.getInstance();
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                wantedState.setOutputs(outputs);
//                if(Robot.currentPhase.equals(Robot.MatchPhase.TELEOP)) {
//                    if (wantedState.equals(LedWantedState.LOADED_STROBE) || wantedState.equals(LedWantedState.HAS_NOTE)) {
//                        vision.blinkLights();
//                    } else {
//                        vision.disableLights();
//                    }
//                }
            }

            @Override
            public void onStop(double timestamp) {
                wantedState = LedWantedState.DISABLED;
            }
        });
    }

    public void setWantedState(LedWantedState wantedState) {
        if(wantedState != null && !wantedState.equals(this.wantedState)) {
            this.wantedState = wantedState;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (wantedState.isAnimation()) {
            candle.animate(outputs.animation);
        } else {
            candle.clearAnimation(0);
            candle.setLEDs(outputs.r, outputs.g, outputs.b);
        }
        
    }

    @Override
    public void writeDisabledPeriodicOutputs() {
        writePeriodicOutputs();
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
    }
}
