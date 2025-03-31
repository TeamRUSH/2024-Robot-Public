package net.teamrush27.frc2024.subsystems.indexer;

import monologue.Logged;
import monologue.Annotations.Log;

public class IndexerIO {


    public static class IndexerInputs implements Logged{
        @Log
        double motorSpeed;
        @Log
        double motorTemperature;
        @Log
        double motorCurrent;
        @Log
        boolean intakeSensorDebounced;
        @Log
        boolean frontSensorDebounced;
        @Log
        boolean backSensorDebounced;
    }


    public static class IndexerOutputs implements Logged {
        @Log
        double indexRPM;
    }
    
}
