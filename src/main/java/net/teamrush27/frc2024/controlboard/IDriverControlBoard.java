package net.teamrush27.frc2024.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IDriverControlBoard {
    double getThrottle();
    double getStrafe();
    double getRotation();
    Trigger getResetHeading();
    Trigger getRobotOriented();
    Trigger getSpeakerTracking();
    Trigger getSourceShot1();
    Trigger getSourceShot2();
    Trigger getCrossFieldShot();
    Trigger getNoteTracking();
    Trigger incrementCrankAngle();
    Trigger decrementCrankAngle();
}