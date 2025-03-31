package net.teamrush27.frc2024.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Logged;
import monologue.Annotations.Log;

public class DrivetrainInputs implements Logged{
    @Log
    public double omegaCommand; // radians/s
    @Log
    public double xCommand; // m/s
    @Log
    public double yCommand; // m/s
    @Log
    public double magnitudeCommand;
    @Log
    public Rotation2d yaw = new Rotation2d();
    @Log
    public Rotation2d yawRate = new Rotation2d();
    @Log
    public double angleToTarget;
    @Log
    public double angleToCrossShot;
    @Log
    public Pose2d robotPose = new Pose2d();
    @Log
    public SwerveModuleState[] moduleStates;
    @Log
    public double[] moduleDriveVelocitiesMps = new double[4];
    @Log
    public double[] moduleDriveCurrentsMps = new double[4];
    @Log
    public double[] moduleDriveSupplyCurrents = new double[4];
    @Log
    public double[] moduleDriveTemps = new double[4];
    @Log
    public double[] moduleSteeringPositions = new double[4];
    @Log
    public double[] moduleSteerErrors = new double[4];
    @Log
    public double robotAngleToField;

    @Log
    public double yawError;
}
