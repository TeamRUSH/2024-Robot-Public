package net.teamrush27.frc2024.subsystems.intake;

import monologue.Logged;
import monologue.Annotations.Log;

public class IntakeInputs implements Logged{
    @Log
    double lampreyPosition;
    @Log
    double filteredLampreyPosition;
    @Log
    double relativePosition;
    @Log
    double deployMotorCurrent;
    @Log
    double rollerMotorCurrent;

    @Log
    double deployMotorAppliedOutput;

}
