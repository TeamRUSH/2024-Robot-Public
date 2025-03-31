package net.teamrush27.frc2024.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Logged;
import monologue.Annotations.Log;

public class DrivetrainOutputs implements Logged{
    @Log
    public ChassisSpeeds trajectorySpeeds = new ChassisSpeeds();
    @Log
    public SwerveModuleState[] targetStates;

}
