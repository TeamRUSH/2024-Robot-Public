package net.teamrush27.frc2024.subsystems.intake;

import monologue.Logged;
import monologue.Annotations.Log;

class IntakeOutputs implements Logged{
    @Log
    double rollerDuty; // RPM
    @Log
    double targetAngle; // Degrees
}
