package net.teamrush27.frc2024.subsystems.ampMech;

import monologue.Logged;
import monologue.Annotations.Log;

public interface AmpMechIO {

    public class Inputs implements Logged{
        @Log
        public double position;
        @Log
        public double current;
        @Log
        public double motorDutyActual;
        @Log
        public double positionError;
    }

    public class Outputs implements Logged{
        @Log
        public double positionCommand;
    }
}
