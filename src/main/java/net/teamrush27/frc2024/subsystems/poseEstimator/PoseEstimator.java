// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.teamrush27.frc2024.subsystems.poseEstimator;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import monologue.Annotations;
import monologue.Annotations.IgnoreLogged;
import net.teamrush27.frc2024.subsystems.drivetrain.Drivetrain;
import net.teamrush27.frc2024.subsystems.drivetrain.RushSwerveDrivetrain;
import net.teamrush27.frc2024.subsystems.poseEstimator.PoseEstimatorIO.PoseEstimatorInputs;
import net.teamrush27.frc2024.subsystems.poseEstimator.PoseEstimatorIO.PoseEstimatorOutputs;
import net.teamrush27.frc2024.subsystems.vision.Vision;
import net.teamrush27.frc2024.util.AprilTag2024Field;
import net.teamrush27.frc2024.util.OneDimensionalLookup;
import net.teamrush27.frc2024.util.RushMath;
import net.teamrush27.frc2024.util.TimedPose3d;
import net.teamrush27.frc2024.util.Vector2d;

import java.util.Optional;

/**
 * Add your docs here.
 */
public class PoseEstimator extends Subsystem {

    @IgnoreLogged
    static PoseEstimator instance;

    public static PoseEstimator getInstance() {
        if (instance == null) {
            instance = new PoseEstimator();
        }
        return instance;
    }

    @IgnoreLogged
    private final Drivetrain drivetrain;
    private final Vision vision = Vision.getInstance();
    private final AprilTag2024Field field = AprilTag2024Field.getInstance();

    // Internal variable declarations
    private final PoseEstimatorInputs inputs;
    private final PoseEstimatorOutputs outputs;
    private final SwerveDriveState drivetrainState;
    private final RushSwerveDrivetrain swerveDrivetrainInstance;
    private Alliance allianceColor = Alliance.Blue;
    private TimedPose3d launcherLimelightTimedPose = new TimedPose3d(new Pose3d(), 0.0);
    private TimedPose3d intakeLimelightTimedPose = new TimedPose3d(new Pose3d(), 0.0);
    private final Debouncer tagAcquiredDebounce = new Debouncer(0.5, Debouncer.DebounceType.kRising);
    private final MedianFilter launcherLimelightFilterX = new MedianFilter(PoseEstimatorConfig.MEDIAN_FILTER_SAMPLES);
    private final MedianFilter launcherLimelightFilterY = new MedianFilter(PoseEstimatorConfig.MEDIAN_FILTER_SAMPLES);
    private final MedianFilter launcherLimelightFilterAngle = new MedianFilter(
            PoseEstimatorConfig.MEDIAN_FILTER_SAMPLES);

    private final MedianFilter xSpeedsFilter = new MedianFilter(3);
    private final MedianFilter ySpeedsFilter = new MedianFilter(3);
    private final MedianFilter omegaSpeedsFilter = new MedianFilter(3);

    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
    private double lastTimestamp = 0;

    public PoseEstimator() {
        drivetrain = Drivetrain.getInstance();
        swerveDrivetrainInstance = Drivetrain.getInstance().getSwerveDrivetrainInstance();
        drivetrainState = swerveDrivetrainInstance.getState();
        inputs = new PoseEstimatorInputs();
        outputs = new PoseEstimatorOutputs();
        vision.init();

    }

    @Override
    public void registerEnabledLoops(ILooper loops) {
        loops.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // setAllianceColor();
            }

            @Override
            public void onLoop(double timestamp) {
                if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) {
                    allianceColor = Alliance.Red;
                } else {
                    allianceColor = Alliance.Blue;
                }

            }

            @Override
            public void onStop(double timestamp) {
            }

        });
    }

    public void readPeriodicInputs() {
        inputs.drivetrainPose = drivetrainState.Pose;

        vision.periodic(inputs.drivetrainPose.getRotation());

        if (vision.checkLimelightBackSeesTag()) {
            inputs.launcherLimelightUpdateAvailable = true;
            if (DriverStation.isDisabled()) {
                launcherLimelightTimedPose = vision.getLauncherMegaTag1Pose();
            } else {
                launcherLimelightTimedPose = vision.getLauncherMegaTag2Pose();
            }
            inputs.launcherLimelightPose = launcherLimelightTimedPose.pose;
            inputs.launcherLimelightPose2d = inputs.launcherLimelightPose.toPose2d();
            inputs.launcherLimelightPoseTimestampPrev = inputs.launcherLimelightPoseTimestamp;
            inputs.launcherLimelightPoseTimestamp = launcherLimelightTimedPose.timestamp;
            inputs.launcherLimelightNumOfTags = vision.getBackLimelightNumberOfFiducials();
            inputs.addLauncherVisionPose = acceptVisionPose();
        } else {
            inputs.launcherLimelightUpdateAvailable = false;
        }

        if (vision.checkLimelightFrontSeesTag()) {
            inputs.intakeLimelightUpdateAvailable = true;
            intakeLimelightTimedPose = vision.getLimelightFrontVisionPose();
            inputs.intakeLimelightPose = intakeLimelightTimedPose.pose;
            // inputs.intakeLimelightPoseTimestamp = timedPose.timestamp;
        } else {
            inputs.intakeLimelightUpdateAvailable = false;
        }
        updateSpeeds();
        inputs.estimatedTimeOfFlight = getDynamicNoteTimeOfFlight();
//        inputs.estimatedTimeOfFlight = getNoteTimeOfFlight();
        inputs.crossShotTimeOfFlight = getCrossShotTimeOfFlight();
    }

    public void writePeriodicOutputs() {

        updatePoseEstimates();
        calcVectors(allianceColor);
        outputTelemetry(false);

        outputs.distanceToTarget = outputs.vectorRobotToTarget.getDistance();

        outputs.futureDistanceToTarget = outputs.vectorFutureRobotToTarget.getDistance();

        outputs.futureDistanceForCrossShot = outputs.vectorFutureRobotToCrossShot.getDistance();
        outputs.distanceForCrossShot = outputs.vectorRobotToCrossShot.getDistance();
        outputs.distanceForSourceShot1 = outputs.vectorRobotToSourceShot1.getDistance();
        outputs.distanceForSourceShot2 = outputs.vectorRobotToSourceShot2.getDistance();

    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    private void updatePoseEstimates() {
        double speedMagnitude = RushMath.pythagorean(
                inputs.fieldRelativeSpeeds.vxMetersPerSecond,
                inputs.fieldRelativeSpeeds.vyMetersPerSecond);

        if (inputs.addLauncherVisionPose) {

            if (DriverStation.isDisabled()) {
                addLauncherLimelightMeasurement(PoseEstimatorConfig.STATIONARY_LL_STDDEV);
            // } else if (DriverStation.isAutonomous()) {
            //     updatePoseInAuton();
            // } else if (inputs.launcherLimelightNumOfTags > 1) {
            //     addLauncherLimelightMeasurement(PoseEstimatorConfig.STATIONARY_LL_STDDEV);
            // } else if (inputs.launcherLimelightNumOfTags == 1) {
            //     addLauncherLimelightMeasurement(PoseEstimatorConfig.STATIONARY_LL_STDDEV);
            // }
            }
            else{
                updatePoseInAuton();
            }

        }
        // if (tagAcquiredDebounce.calculate(inputs.intakeLimelightUpdateAvailable)) {
        // swerveDrivetrainInstance.addVisionMeasurement(
        // inputs.intakeLimelightPose.toPose2d(),
        // intakeLimelightTimedPose.timestamp,
        // PoseEstimatorConfig.INTAKE_LL_STDDEV);
        // }
    }

    private boolean isRobotMoving() {
        double threshold = 0.5;
        return inputs.fieldRelativeSpeeds.vxMetersPerSecond > threshold
                || inputs.fieldRelativeSpeeds.vyMetersPerSecond > threshold;
    }

    private void calcVectors(Alliance colorAlliance) {

        switch (colorAlliance) {
            case Red:
                outputs.aimPoint = field.getTagPose3d(4).plus(
                        new Transform3d(PoseEstimatorConfig.AIMPOINT_X_OFFSET, PoseEstimatorConfig.AIMPOINT_Y_OFFSET, 0,
                                new Rotation3d()));
                outputs.vectorRobotToTarget = new Vector2d(inputs.drivetrainPose, outputs.aimPoint.toPose2d());
                outputs.vectorFutureRobotToTarget = new Vector2d(outputs.futurePose,
                        field.getTagPose3d(4).toPose2d());
                outputs.vectorRobotToAmp = new Vector2d(inputs.drivetrainPose, field.getTagPose3d(5).toPose2d());
                outputs.vectorRobotToSourceClose = new Vector2d(inputs.drivetrainPose,
                        field.getTagPose3d(10).toPose2d());
                outputs.vectorRobotToSourceFar = new Vector2d(inputs.drivetrainPose,
                        field.getTagPose3d(9).toPose2d());

                outputs.crossShotTarget = new Pose2d(Units.feetToMeters(54) - 1.5, 7.2, new Rotation2d());
                outputs.sourceShot1Target = new Pose2d(Units.feetToMeters(27) + 1.3, 7.5, new Rotation2d());
                outputs.sourceShot2Target = new Pose2d(Units.feetToMeters(27) /*+ 1.3*/, 4.5, new Rotation2d());

                outputs.vectorRobotToCrossShot = new Vector2d(inputs.drivetrainPose, outputs.crossShotTarget);
                outputs.vectorRobotToSourceShot1 = new Vector2d(inputs.drivetrainPose, outputs.sourceShot1Target);
                outputs.vectorRobotToSourceShot2 = new Vector2d(inputs.drivetrainPose, outputs.sourceShot2Target);

                outputs.vectorFutureRobotToCrossShot = new Vector2d(outputs.futurePoseForCrossShot, outputs.crossShotTarget);
                break;
            case Blue:
            default:
                outputs.aimPoint = field.getTagPose3d(7).plus(
                        new Transform3d(PoseEstimatorConfig.AIMPOINT_X_OFFSET, PoseEstimatorConfig.AIMPOINT_Y_OFFSET, 0,
                                new Rotation3d()));
                outputs.vectorRobotToTarget = new Vector2d(inputs.drivetrainPose, outputs.aimPoint.toPose2d());
                outputs.vectorFutureRobotToTarget = new Vector2d(outputs.futurePose,
                        field.getTagPose3d(7).toPose2d());
                outputs.vectorRobotToAmp = new Vector2d(inputs.drivetrainPose, field.getTagPose3d(6).toPose2d());
                outputs.vectorRobotToSourceClose = new Vector2d(inputs.drivetrainPose,
                        field.getTagPose3d(1).toPose2d());
                outputs.vectorRobotToSourceFar = new Vector2d(inputs.drivetrainPose,
                        field.getTagPose3d(2).toPose2d());

                outputs.crossShotTarget = new Pose2d(1.5, 6.4, new Rotation2d()); // 1, 6
                outputs.sourceShot1Target = new Pose2d(Units.feetToMeters(27) - 1.3, 7.0, new Rotation2d());
                outputs.sourceShot2Target = new Pose2d(Units.feetToMeters(27) - 1.3, 3.5, new Rotation2d());

                outputs.vectorRobotToCrossShot = new Vector2d(inputs.drivetrainPose, outputs.crossShotTarget);
                outputs.vectorRobotToSourceShot1 = new Vector2d(inputs.drivetrainPose, outputs.sourceShot1Target);
                outputs.vectorRobotToSourceShot2 = new Vector2d(inputs.drivetrainPose, outputs.sourceShot2Target);

                outputs.vectorFutureRobotToCrossShot = new Vector2d(outputs.futurePoseForCrossShot, outputs.crossShotTarget);
                break;

        }

//        if(vision.getNoteTy().isPresent() && vision.getNoteTx().isPresent()) {
//            getTransformToNote().ifPresent(transform2d -> outputs.transformToNote = transform2d);
//            outputs.notePose = inputs.drivetrainPose.plus(outputs.transformToNote);
//        }


        outputs.futurePose = inputs.drivetrainPose
                    .plus(
                            new Transform2d(
                                    inputs.fieldRelativeSpeeds.vxMetersPerSecond,
                                    inputs.fieldRelativeSpeeds.vyMetersPerSecond,
                                    Rotation2d.fromRadians(0)
                            ).times(inputs.estimatedTimeOfFlight).times(PoseEstimatorConfig.NOTE_FLIGHT_TIME_SCALAR));

        // if (isRobotMoving()) {
        //     outputs.futurePose = inputs.drivetrainPose
        //             .plus(
        //                     new Transform2d(
        //                             inputs.fieldRelativeSpeeds.vxMetersPerSecond,
        //                             inputs.fieldRelativeSpeeds.vyMetersPerSecond,
        //                             Rotation2d.fromRadians(0)).times(inputs.estimatedTimeOfFlight));
        // } else {
        //     outputs.futurePose = inputs.drivetrainPose;
        // }

        outputs.futurePoseForCrossShot = inputs.drivetrainPose
                .plus(
                        new Transform2d(
                                inputs.fieldRelativeSpeeds.vxMetersPerSecond,
                                inputs.fieldRelativeSpeeds.vyMetersPerSecond,
                                Rotation2d.fromRadians(0)
                        ).times(0.5));

    }

    private void setAllianceColor() {
        allianceColor = Alliance.Blue;
    }

    public boolean checkVisionTargetsAvailable() {
        return inputs.launcherLimelightUpdateAvailable;
    }

    public Pose2d getVisionPose() {
        return inputs.launcherLimelightPose.toPose2d();
    }

    public void resetDrivetrainOdometry() {
        if (inputs.launcherLimelightUpdateAvailable) {
            swerveDrivetrainInstance.seedFieldRelative(vision.getLauncherMegaTag1Pose().pose.toPose2d());
            swerveDrivetrainInstance.setFieldRelativeOffset();
        }
    }

    private boolean acceptVisionPose() {
        return true;
    }

    private void addLauncherLimelightMeasurement(Matrix<N3, N1> stdDev) {
        outputs.filteredVisionPose = inputs.launcherLimelightPose;
        swerveDrivetrainInstance.addVisionMeasurement(
                inputs.launcherLimelightPose.toPose2d(),
                launcherLimelightTimedPose.timestamp,
                stdDev);
    }

    private void addVariableLauncherLimelightMeasurement() {
        // Use limelight distance to increase Vector numbers as distance increases
        double distance = inputs.launcherLimelightPose.getX();
        // Make value relative to distance where close is 1.7 and far is 4
        double standard_deviation = distance <= 0 ? .5 : (distance <= 7 ? .5 + (distance / 7) * (4.0 - .5) : 4.0);
        Matrix<N3, N1> variable_Matrix = VecBuilder.fill(standard_deviation, standard_deviation, 1.0);
        swerveDrivetrainInstance.addVisionMeasurement(
                inputs.launcherLimelightPose.toPose2d(),
                launcherLimelightTimedPose.timestamp,
                variable_Matrix);
    }

    public double getDistanceToTarget() {
        return Units.metersToFeet(outputs.distanceToTarget);
    }

    public double get3dDistanceToTarget() {
        return RushMath.pythagorean(
                outputs.vectorRobotToTarget.getTranslation2d().getX(),
                outputs.vectorRobotToTarget.getTranslation2d().getY(),
                1.5);
    }

    public double getFutureDistanceToTarget() {
        return Units.metersToFeet(outputs.futureDistanceToTarget);
    }

    public Vector2d getVectorToTarget() {
        return outputs.vectorRobotToTarget;
    }

    @Deprecated
    public Rotation2d getRotationToTarget() {
        if (allianceColor.equals(Alliance.Red)) {

            return outputs.vectorRobotToTarget.getRotation()
                    .rotateBy(new Rotation2d(Math.PI))
                    .rotateBy(PoseEstimatorConfig.AIM_YAW_OFFSET_RED);
        } else {
            return outputs.vectorRobotToTarget.getRotation()
                    .rotateBy(PoseEstimatorConfig.AIM_YAW_OFFSET_BLUE);
        }
    }

    public Rotation2d getFutureRotationToTarget() {
        double angleOffset;
        if (allianceColor.equals(Alliance.Red)) {
            angleOffset = OneDimensionalLookup.interpLinear(PoseEstimatorConfig.AIM_OFFSET_RED_ANGLES,
                    PoseEstimatorConfig.AIM_OFFSET_RED_OFFSETS,
                    outputs.vectorFutureRobotToTarget.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI))
                            .getRadians());
            return outputs.vectorFutureRobotToTarget.getRotation()
                    .rotateBy(Rotation2d.fromRadians(Math.PI))
                    .rotateBy(Rotation2d.fromDegrees(2));

        } else {
            angleOffset = OneDimensionalLookup.interpLinear(PoseEstimatorConfig.AIM_OFFSET_BLUE_ANGLES,
                    PoseEstimatorConfig.AIM_OFFSET_BLUE_OFFSETS,
                    outputs.vectorFutureRobotToTarget.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI))
                            .getRadians());
            return outputs.vectorFutureRobotToTarget.getRotation()
                    .rotateBy(Rotation2d.fromRadians(Math.PI))
                    .rotateBy(Rotation2d.fromDegrees(1));
        }
    }

    public Rotation2d getDrivebaseAngleToField() {
        return inputs.drivetrainPose.getRotation();
    }

    @Annotations.Log
    double avgNoteVelocity;

    @Annotations.Log
    double velocityTowardSpeaker;

    public double getDynamicNoteTimeOfFlight() {
        avgNoteVelocity = outputs.distanceToTarget / OneDimensionalLookup.interpLinear(
                PoseEstimatorConfig.DISTANCE_BREAKPOINTS,
                PoseEstimatorConfig.NOTE_FLIGHT_TIMES,
                outputs.distanceToTarget);

        velocityTowardSpeaker = (inputs.fieldRelativeSpeeds.vxMetersPerSecond
                * outputs.vectorRobotToTarget.getTranslation2d().getX()
                + inputs.fieldRelativeSpeeds.vyMetersPerSecond * outputs.vectorRobotToTarget.getTranslation2d().getY())
                / outputs.distanceToTarget;

        double finalNoteVelocity = avgNoteVelocity + velocityTowardSpeaker;

        return outputs.distanceToTarget / finalNoteVelocity;
    }

    public double getNoteTimeOfFlight() {
        return OneDimensionalLookup.interpLinear(
                PoseEstimatorConfig.DISTANCE_BREAKPOINTS,
                PoseEstimatorConfig.NOTE_FLIGHT_TIMES,
                outputs.distanceToTarget
                );
    }

    public double getCrossShotTimeOfFlight() {
        return OneDimensionalLookup.interpLinear(
                PoseEstimatorConfig.CROSS_SHOT_DISTANCE_BREAKPOINTS,
                PoseEstimatorConfig.CROSS_SHOT_NOTE_FLIGHT_TIMES,
                getDistanceForCrossShot());
    }

    private void updatePoseInAuton() {
        double speed = RushMath.pythagorean(drivetrainState.speeds.vxMetersPerSecond,
                drivetrainState.speeds.vyMetersPerSecond);
        double distance = vision.getDistanceToClosestTag();
        double yawRate = Math.abs(drivetrainState.speeds.omegaRadiansPerSecond);
        double poseX = drivetrainState.Pose.getX();

        // if (poseX > 4.5 && poseX < 12.5){
        // return;
        // }
        if (yawRate < 4 * Math.PI){
            if (distance < 22) {
                swerveDrivetrainInstance.addVisionMeasurement(
                    inputs.launcherLimelightPose.toPose2d(),
                    launcherLimelightTimedPose.timestamp,
                    PoseEstimatorConfig.AUTON_LL_STDDEV);
                } else if (distance < 30) {
                    swerveDrivetrainInstance.addVisionMeasurement(
                        inputs.launcherLimelightPose.toPose2d(),
                        launcherLimelightTimedPose.timestamp,
                        PoseEstimatorConfig.STD_DEV_SINGLE_TAG);
                }   
        }
        // if(speed < 1.2 && distance < 5.0){
        // swerveDrivetrainInstance.addVisionMeasurement(
        // inputs.launcherLimelightPose.toPose2d(),
        // launcherLimelightTimedPose.timestamp,
        // PoseEstimatorConfig.STATIONARY_LL_STDDEV);
        // }

        // if(inputs.launcherLimelightNumOfTags > 1){
        // //Higher level of trust but decrease with distance, do not update at high
        // speed

        // vision.enableLights();
        // swerveDrivetrainInstance.addVisionMeasurement(
        // inputs.launcherLimelightPose.toPose2d(),
        // launcherLimelightTimedPose.timestamp,
        // PoseEstimatorConfig.STATIONARY_LL_STDDEV);
        // } else if(distance < 4.5){
        // vision.disableLights();
        // swerveDrivetrainInstance.addVisionMeasurement(
        // inputs.launcherLimelightPose.toPose2d(),
        // launcherLimelightTimedPose.timestamp,
        // PoseEstimatorConfig.AUTON_LL_STDDEV);
        // }

        // } else if(inputs.launcherLimelightNumOfTags == 1){
        // vision.enableLights();

        // //Lower trust, weak update only when not moving
        // if(speed < 0.25 && distance < 3.0){
        // swerveDrivetrainInstance.addVisionMeasurement(
        // inputs.launcherLimelightPose.toPose2d(),
        // launcherLimelightTimedPose.timestamp,
        // PoseEstimatorConfig.STATIONARY_LL_STDDEV);
        // }
        // }
        // Otherwise do not update pose
    }

    public double getDistanceForCrossShot() {
        return Units.metersToFeet(outputs.distanceForCrossShot);
    }

    public double getDistanceForSourceShot1() {
        return Units.metersToFeet(outputs.distanceForSourceShot1);
    }

    public double getDistanceForSourceShot2() {
        return Units.metersToFeet(outputs.distanceForSourceShot2);
    }

    public Rotation2d getRotationForCrossShot() {
        if (allianceColor.equals(Alliance.Red)) {
            return outputs.vectorFutureRobotToCrossShot.getRotation()
                    .rotateBy(Rotation2d.fromRadians(Math.PI))
                    .rotateBy(PoseEstimatorConfig.AIM_YAW_OFFSET_RED);
        } else {
            return outputs.vectorFutureRobotToCrossShot.getRotation()
                    .rotateBy(Rotation2d.fromRadians(Math.PI))
                    .rotateBy(PoseEstimatorConfig.AIM_YAW_OFFSET_BLUE);
        }

    }
    public Rotation2d getRotationForSourceShot1() {
        if(allianceColor.equals(Alliance.Red)) {
            return outputs.vectorRobotToSourceShot1.getRotation()
                    .rotateBy(Rotation2d.fromRadians(Math.PI))
                    .rotateBy(PoseEstimatorConfig.AIM_YAW_OFFSET_RED);
        } else {
            return outputs.vectorRobotToSourceShot1.getRotation()
                    .rotateBy(Rotation2d.fromRadians(Math.PI))
                    .rotateBy(PoseEstimatorConfig.AIM_YAW_OFFSET_BLUE);
        }
    }

    public Rotation2d getRotationForSourceShot2() {
        if(allianceColor.equals(Alliance.Red)) {
            return outputs.vectorRobotToSourceShot2.getRotation()
                    .rotateBy(Rotation2d.fromRadians(Math.PI))
                    .rotateBy(PoseEstimatorConfig.AIM_YAW_OFFSET_RED);
        } else {
            return outputs.vectorRobotToSourceShot2.getRotation()
                    .rotateBy(Rotation2d.fromRadians(Math.PI))
                    .rotateBy(PoseEstimatorConfig.AIM_YAW_OFFSET_BLUE);
        }

    }

    public double getFutureDistanceForCrossShot() {
        return Units.metersToFeet(outputs.futureDistanceForCrossShot);
    }

    private void updateSpeeds() {
        double timestamp = Timer.getFPGATimestamp();
        // filter the actual speeds to smooth the speeds used for future pose estimation
        inputs.fieldRelativeSpeeds = new ChassisSpeeds(
                xSpeedsFilter.calculate(drivetrainState.speeds.vxMetersPerSecond),
                ySpeedsFilter.calculate(drivetrainState.speeds.vyMetersPerSecond),
                omegaSpeedsFilter.calculate(drivetrainState.speeds.omegaRadiansPerSecond));

        inputs.fieldRelativeAccelerations = inputs.fieldRelativeSpeeds
                .minus(lastSpeeds)
                .div(timestamp - lastTimestamp);

        lastSpeeds = inputs.fieldRelativeSpeeds;
        lastTimestamp = timestamp;
    }

    public Optional<Transform2d> getTransformToNote() {
        if(vision.getNoteTx().isPresent() && vision.getNoteTy().isPresent()) {
            Rotation2d tx = Rotation2d.fromDegrees(vision.getNoteTx().get());
            Rotation2d ty = Rotation2d.fromDegrees(vision.getNoteTy().get());

            double r = PoseEstimatorConfig.NOTE_CAMERA_HEIGHT_M / Math.tan(PoseEstimatorConfig.NOTE_CAMERA_ANGLE.getRadians() + ty.getRadians());

            double x = r * Math.cos(tx.getRadians());
            double y = r * Math.sin(tx.getRadians());

            return Optional.of(new Transform2d(x,y, tx));
        } else {
            return Optional.empty();
        }
    }

    private boolean checkPoseInFieldBound(Pose2d pose) {
        return pose.getX() >= 0 && pose.getX() <= 16.5d && pose.getY() >= 0 && pose.getY() <= 8.25d;
    }

    public Pose2d getNotePose() {
        return outputs.notePose;
    }

}
