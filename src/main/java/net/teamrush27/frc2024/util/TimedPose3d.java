package net.teamrush27.frc2024.util;

import edu.wpi.first.math.geometry.Pose3d;

public class TimedPose3d {

    public Pose3d pose;
    public double timestamp;

    public TimedPose3d(Pose3d pose, double timestamp){
        this.pose = pose;
        this.timestamp = timestamp;
    }
}
