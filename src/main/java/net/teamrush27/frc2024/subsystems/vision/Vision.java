package net.teamrush27.frc2024.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import net.teamrush27.frc2024.util.*;

import javax.swing.plaf.basic.BasicSplitPaneUI;
import java.util.Optional;

public class Vision {

    private final static String LIMELIGHT_BACK = "limelight-back";
    private final static String LIMELIGHT_FRONT = "limelight-front";
    private Transform3d LIMELIGHT_BACK_TRANSFORM = new Transform3d(-0.166, 0, 0.648, new Rotation3d(0, 20, 180));

    private final double SETUP_TOLERANCE = Units.inchesToMeters(1);

    @Log
    private VisionWantedState wantedState = VisionWantedState.APRILTAG;
    @Log
    private VisionSystemState currentState = VisionSystemState.APRILTAG;

    @IgnoreLogged
    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public void setWantedState(VisionWantedState wantedState) {
        this.wantedState = wantedState;
    }


    public void init() {
        // NewLimelightHelpers.setCameraPose_RobotSpace(
        //         LIMELIGHT_BACK,
        //         LIMELIGHT_BACK_TRANSFORM.getX(),
        //         LIMELIGHT_BACK_TRANSFORM.getY(),LIMELIGHT_BACK_TRANSFORM.getZ(),
        //         LIMELIGHT_BACK_TRANSFORM.getRotation().getX(),
        //         LIMELIGHT_BACK_TRANSFORM.getRotation().getY(),
        //         LIMELIGHT_BACK_TRANSFORM.getRotation().getZ()
        // );
    }

    public void periodic(Rotation2d rotation) {
        NewLimelightHelpers.SetRobotOrientation(LIMELIGHT_BACK, rotation.getDegrees(),0,0,0,0,0);
    }

    private TimedPose2d getVisionPose(String limelightName) {
        // Default to Blue if we don't have one
        // DriverStation.Alliance alliance =
        // DriverStation.getAlliance().orElse(Alliance.Blue);
        DriverStation.Alliance alliance = Alliance.Blue;
        Pose2d botpose = switch (alliance) {
            case Red -> LimelightHelpers.getBotPose2d_wpiRed(limelightName);
            case Blue -> LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
        };
        return new TimedPose2d(botpose,
                Timer.getFPGATimestamp() - ((LimelightHelpers.getLatency_Pipeline(limelightName) +
                        LimelightHelpers.getLatency_Capture(limelightName)) / 1000));
    }

    private TimedPose3d getMegaTag2Pose(String limelightName) {
        Pose2d botpose2d = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName).pose;
        Pose3d botpose = new Pose3d(botpose2d);
        return new TimedPose3d(botpose,
                Timer.getFPGATimestamp() - ((LimelightHelpers.getLatency_Pipeline(limelightName) +
                        LimelightHelpers.getLatency_Capture(limelightName)) / 1000));
    }

    private TimedPose3d getMegaTag1Pose(String limelightName) {
        Pose3d botpose = LimelightHelpers.getBotPose3d_wpiBlue(limelightName);
        return new TimedPose3d(botpose,
                Timer.getFPGATimestamp() - ((LimelightHelpers.getLatency_Pipeline(limelightName) +
                        LimelightHelpers.getLatency_Capture(limelightName)) / 1000));
    }

    public TimedPose3d getLauncherMegaTag2Pose() {
        return getMegaTag2Pose(LIMELIGHT_BACK);
    }

    public TimedPose3d getLauncherMegaTag1Pose() {
        return getMegaTag1Pose(LIMELIGHT_BACK);
    }

    public TimedPose3d getLimelightFrontVisionPose() {
        return getMegaTag2Pose(LIMELIGHT_FRONT);
    }

    public double[] getBotposeTargetSpace() {
        double[] botposeArray = LimelightHelpers.getBotPose_TargetSpace(LIMELIGHT_BACK);

        return botposeArray;
    }

    public boolean checkLimelightBackSeesTag() {
        return LimelightHelpers.getTV(LIMELIGHT_BACK);
    }

    public boolean checkLimelightFrontSeesTag() {
        return LimelightHelpers.getTV(LIMELIGHT_FRONT);
    }

    public int getBackLimelightNumberOfFiducials() {
        return LimelightHelpers.getFiducialArraySize(LIMELIGHT_BACK);
    }

    public void enableLights() {
        LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_BACK);
        LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_FRONT);
    }

    public void disableLights() {
        LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_BACK);
        LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_FRONT);
    }

    public double getDistanceToClosestTag() {
        Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace(LIMELIGHT_BACK);
        return RushMath.pythagorean(target.getX(), target.getY());
    }

    public Optional<Double> getNoteTx() {
        if(LimelightHelpers.getTV(LIMELIGHT_FRONT)) {
            return Optional.of(LimelightHelpers.getTX(LIMELIGHT_FRONT));
        } else {
            return Optional.empty();
        }
    }

    public Optional<Double> getNoteTy() {
        if(LimelightHelpers.getTV(LIMELIGHT_FRONT)) {
            return Optional.of(LimelightHelpers.getTY(LIMELIGHT_FRONT));
        } else {
            return Optional.empty();
        }
    }

    public void blinkLights() {
        LimelightHelpers.setLEDMode_ForceBlink(LIMELIGHT_FRONT);
        LimelightHelpers.setLEDMode_ForceBlink(LIMELIGHT_BACK);
    }

    public boolean seesNote() {
        return LimelightHelpers.getTV(LIMELIGHT_FRONT);
    }

    public boolean inSourceStartingPosition() {
        if (getLauncherMegaTag2Pose() == null) {
            return false;
        }
        Pose3d pose = getLauncherMegaTag2Pose().pose;
        if (Math.abs(pose.getX() - 1.44) <= SETUP_TOLERANCE && Math.abs(pose.getY() - 1.73) <= SETUP_TOLERANCE) {
            return true;
        }
        return false;
    }

    public boolean inAmpStartingPosition() {
        if (getLauncherMegaTag2Pose() == null) {
            return false;
        }
        Pose3d pose = getLauncherMegaTag2Pose().pose;
        if (Math.abs(pose.getX() - 1.44) <= SETUP_TOLERANCE && Math.abs(pose.getY() - 1.73) <= SETUP_TOLERANCE) {
            return true;
        }
        return false;
    }
}
