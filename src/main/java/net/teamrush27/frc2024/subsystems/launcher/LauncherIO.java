// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.teamrush27.frc2024.subsystems.launcher;

import monologue.Logged;
import monologue.Annotations.Log;

/** Add your docs here. */
public interface LauncherIO {

    class LauncherInputs implements Logged{
        @Log
        double leftMotorClosedLoopError;
        @Log
        double leftMotorSpeed;
        @Log
        double leftMotorCurrent;
        @Log
        double leftMotorClosedLoopReference;
        @Log
        double rightMotorClosedLoopError;
        @Log
        double rightMotorSpeed;
        @Log
        double rightMotorCurrent;
        @Log
        double rightMotorClosedLoopReference;
        @Log
        double crankAngleError;
        @Log
        double crankAngle;
        @Log
        double currentDistanceToTarget;
        @Log
        double futureDistanceToTarget;

        @Log
        double distanceForCrossShot;
        @Log
        double distanceForSourceShot1;
        @Log
        double distanceForSourceShot2;

        @Log
        double[] speakerCrankAngles;

        @Log
        boolean crankAngleSensorFault;
    }


    class LauncherOutputs implements Logged {
        @Log
        double leftRollerSpeedTarget;
        @Log
        double rightRollerSpeedTarget;
        @Log
        double crankAngleTarget;
        
    }
}
