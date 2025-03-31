package net.teamrush27.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;

public class TimedPose2d{

    public Pose2d pose;
    public double timestamp;

    public TimedPose2d(Pose2d pose, double timestamp){
        this.pose = pose;
        this.timestamp = timestamp;
    }
}