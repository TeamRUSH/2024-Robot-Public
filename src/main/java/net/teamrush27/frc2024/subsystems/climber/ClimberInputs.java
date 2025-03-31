package net.teamrush27.frc2024.subsystems.climber;

import monologue.Logged;
import monologue.Annotations.Log;

class ClimberInputs implements Logged{
    @Log
    double motorSpeed;
    @Log
    double motorPosition;
    @Log
    double motorCurrent;
    @Log
    double motorOutputVoltage;
    @Log
    double medianMotorCurrent;
    @Log
    boolean bottomSensor;
    @Log
    boolean topSensor;

}
