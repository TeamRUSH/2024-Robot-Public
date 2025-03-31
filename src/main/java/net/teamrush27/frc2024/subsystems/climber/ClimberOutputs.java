package net.teamrush27.frc2024.subsystems.climber;

import com.revrobotics.CANSparkBase.*;

import monologue.Logged;
import monologue.Annotations.Log;

public class ClimberOutputs implements Logged {
    @Log
    double servoSetpoint;
    @Log
    double motorSetpoint;
    @Log
    ControlType controlType;
}