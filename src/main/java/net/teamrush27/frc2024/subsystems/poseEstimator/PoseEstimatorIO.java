package net.teamrush27.frc2024.subsystems.poseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import monologue.Annotations.Log;
import monologue.Logged;
import net.teamrush27.frc2024.util.Vector2d;

public interface PoseEstimatorIO {

    public static class PoseEstimatorInputs implements Logged {
        @Log
        public Pose2d drivetrainPose = new Pose2d();
        @Log
        public Pose3d launcherLimelightPose = new Pose3d();
        @Log
        public Pose2d launcherLimelightPose2d = new Pose2d();
        //public double launcherLimelightPoseTimestamp;
        @Log
        public Pose3d intakeLimelightPose = new Pose3d();
        //public double intakeLimelightPoseTimestamp;
        @Log
        public boolean launcherLimelightUpdateAvailable = false;
        @Log
        public boolean intakeLimelightUpdateAvailable = false;
        @Log
        public double launcherLimelightPoseTimestamp = 0.0;
        @Log
        public double launcherLimelightPoseTimestampPrev = 0.0;
        @Log
        public ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(); // the change in position after 1 second of the robot's current velocity
        @Log
        public ChassisSpeeds fieldRelativeAccelerations = new ChassisSpeeds(); // the change in velocity after 1 second of the robot's current acceleration
        @Log
        public double estimatedTimeOfFlight;
        @Log
        public double crossShotTimeOfFlight;
        @Log
        public int launcherLimelightNumOfTags = 0;
        @Log
        public int intakeLimelightNumOfTags = 0;
        @Log
        public boolean addLauncherVisionPose = false;


    }


    public static class PoseEstimatorOutputs implements Logged{
        @Log
        public Pose3d filteredVisionPose = new Pose3d();
//        @Log
        public Vector2d vectorRobotToTarget = new Vector2d();
        @Log
        public double distanceToTarget; // feet
//        @Log
        public Vector2d vectorFutureRobotToTarget = new Vector2d();
        @Log
        public double futureDistanceToTarget;
//        @Log
        public Vector2d vectorRobotToAmp = new Vector2d();
//        @Log
        public Vector2d vectorRobotToTrap = new Vector2d();
//        @Log
        public Vector2d vectorRobotToSourceClose = new Vector2d();
//        @Log
        public Vector2d vectorRobotToSourceFar = new Vector2d();
        @Log
        public Rotation2d angleRobotLongitudinalAxisToTarget = new Rotation2d();
        @Log
        public Pose2d futurePose = new Pose2d();
        @Log
        public Pose2d futurePoseForCrossShot = new Pose2d();
        @Log
        public Pose3d aimPoint = new Pose3d();

//        @Log
        public Vector2d vectorRobotToCrossShot = new Vector2d();
        public Vector2d vectorFutureRobotToCrossShot = new Vector2d();

        public Vector2d vectorRobotToSourceShot1 = new Vector2d();
        public Vector2d vectorRobotToSourceShot2 = new Vector2d();

        @Log
        public double distanceForCrossShot;
        @Log
        public double futureDistanceForCrossShot;

        @Log
        public double distanceForSourceShot1;
        @Log
        public double distanceForSourceShot2;

        @Log
        public Pose2d crossShotTarget;

        @Log
        public Pose2d sourceShot1Target;
        @Log
        public Pose2d sourceShot2Target;

        @Log
        public Transform2d transformToNote;

        @Log
        public Pose2d notePose;

    }
}
