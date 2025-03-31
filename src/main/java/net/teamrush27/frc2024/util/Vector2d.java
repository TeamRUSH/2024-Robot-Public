package net.teamrush27.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vector2d extends Translation2d {
    // Takes 2 Pose2d's and returns the distance and angle without regard to the origin or target's Rotations

    private Pose2d originPose;
    private Pose2d targetPose;
    private Translation2d originToTarget;

    public Vector2d(){
        this.originPose = new Pose2d();
        this.targetPose = new Pose2d();
        this.originToTarget = targetPose.relativeTo(originPose).getTranslation();
    }
    public Vector2d(Pose2d origin, Pose2d target){
        this.originPose = new Pose2d(origin.getTranslation(), new Rotation2d());
        this.targetPose = new Pose2d(target.getTranslation(), new Rotation2d());
        this.originToTarget = targetPose.relativeTo(originPose).getTranslation();
    }

    public Translation2d getTranslation2d(){ return originToTarget; }

    public Rotation2d getRotation(){
        return originToTarget.getAngle();
    }

    public double getDistance(){
        return originToTarget.getNorm();
    }
}
