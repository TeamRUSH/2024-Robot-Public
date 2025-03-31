// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.teamrush27.frc2024.subsystems.launcher;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2024.subsystems.drivetrain.Drivetrain;
import net.teamrush27.frc2024.subsystems.leds.Led;
import net.teamrush27.frc2024.subsystems.leds.LedWantedState;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import net.teamrush27.frc2024.subsystems.poseEstimator.PoseEstimator;
import net.teamrush27.frc2024.subsystems.vision.Vision;
import net.teamrush27.frc2024.util.NoteSubsystem;
import net.teamrush27.frc2024.util.OneDimensionalLookup;

public class Launcher extends NoteSubsystem {

    @IgnoreLogged
    static Launcher instance;

    public static Launcher getInstance() {
        if (instance == null) {
            instance = new Launcher();
        }
        return instance;
    }

    private final LauncherIO.LauncherInputs inputs = new LauncherIO.LauncherInputs();
    private final LauncherIO.LauncherOutputs outputs = new LauncherIO.LauncherOutputs();

    private final TalonFX leftRollerMotor;
    private final TalonFX rightRollerMotor;
    private final VelocityVoltage rollerSpeedRequest = new VelocityVoltage(0).withSlot(0);

    private final StatusSignal<Double> leftMotorClosedLoopErrorSignal;
    private final StatusSignal<Double> leftMotorSpeedSignal;
    private final StatusSignal<Double> leftMotorCurrentSignal;
    private final StatusSignal<Double> leftMotorClosedLoopReferenceSignal;
    private final StatusSignal<Double> rightMotorClosedLoopErrorSignal;
    private final StatusSignal<Double> rightMotorSpeedSignal;
    private final StatusSignal<Double> rightMotorCurrentSignal;
    private final StatusSignal<Double> rightMotorClosedLoopReferenceSignal;

    private final CANSparkFlex crankMotor;
    private final SparkPIDController crankPIDController;
    private final SparkAbsoluteEncoder crankThruBoreEncoder;

    public enum ShotType {
        SUBWOOFER,
        SPEAKER,
        AMP,
        DISTANCE_OVERRIDE,
        MANUAL,
        YEET,
        MOONSHOT,
        SOURCE_SHOT_1,
        SOURCE_SHOT_2,
        QUICKSHOT
    }

    @Log
    private LauncherWantedState wantedState = LauncherWantedState.IDLE;
    @Log
    private LauncherSystemState currentState = LauncherSystemState.IDLE;
    @Log
    private ShotType shotType = ShotType.SUBWOOFER;

    @Log
    // private double crankIncrementer = 3.0; after micmp2 q35 changed to 3.75
//    private double crankIncrementer = 2.25d;
    private double crankIncrementer = 3d;

    private double manualDistanceToTarget = 20.0;
    private double manualLeftRollerSpeed = 0.0;
    private double manualRightRollerSpeed = 0.0;
    private double manualCrankAngle = 90.0;

    private double[] crankAngleValues;

    @Log
    private boolean autonQuickshotEnabled = false;
    MedianFilter filter = new MedianFilter(15);

    public Launcher() {
        LauncherConfig.setRobot();
        leftRollerMotor = new TalonFX(LauncherConfig.LEFT_ROLLER_CAN_ID, LauncherConfig.ROLLER_CANBUS);
        rightRollerMotor = new TalonFX(LauncherConfig.RIGHT_ROLLER_CAN_ID, LauncherConfig.ROLLER_CANBUS);

        leftRollerMotor.getConfigurator().apply(LauncherConfig.leftRollerConfig);
        rightRollerMotor.getConfigurator().apply(LauncherConfig.rightRollerConfig);

        leftMotorClosedLoopErrorSignal = leftRollerMotor.getClosedLoopError();
        leftMotorSpeedSignal = leftRollerMotor.getVelocity();
        leftMotorCurrentSignal = leftRollerMotor.getStatorCurrent();
        leftMotorClosedLoopReferenceSignal = leftRollerMotor.getClosedLoopReference();
        rightMotorClosedLoopErrorSignal = rightRollerMotor.getClosedLoopError();
        rightMotorSpeedSignal = rightRollerMotor.getVelocity();
        rightMotorCurrentSignal = rightRollerMotor.getStatorCurrent();
        rightMotorClosedLoopReferenceSignal = rightRollerMotor.getClosedLoopReference();

        leftMotorClosedLoopErrorSignal.setUpdateFrequency(200, 0.05);
        leftMotorSpeedSignal.setUpdateFrequency(200, 0.05);
        leftMotorCurrentSignal.setUpdateFrequency(200, 0.05);
        leftMotorClosedLoopReferenceSignal.setUpdateFrequency(200, 0.05);
        rightMotorClosedLoopErrorSignal.setUpdateFrequency(200, 0.05);
        rightMotorSpeedSignal.setUpdateFrequency(200, 0.05);
        rightMotorCurrentSignal.setUpdateFrequency(200, 0.05);
        rightMotorClosedLoopReferenceSignal.setUpdateFrequency(200, 0.05);

        crankMotor = new CANSparkFlex(LauncherConfig.CRANK_CAN_ID, MotorType.kBrushless);
        crankMotor.clearFaults();
        crankThruBoreEncoder = crankMotor.getAbsoluteEncoder(Type.kDutyCycle);

        crankMotor.setIdleMode(IdleMode.kBrake);
        crankMotor.setInverted(false);
        crankMotor.setSmartCurrentLimit(30);
        crankMotor.setSoftLimit(SoftLimitDirection.kForward, LauncherConfig.CRANK_POSITION_UPPER_LIMIT);
        crankMotor.setSoftLimit(SoftLimitDirection.kReverse, LauncherConfig.CRANK_POSITION_LOWER_LIMIT);
        crankMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        crankMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        crankPIDController = crankMotor.getPIDController();
        crankPIDController.setP(LauncherConfig.CRANK_KP, 0);
        crankPIDController.setI(0, 0);
        crankPIDController.setD(0, 0);
        crankPIDController.setDFilter(1, 0);
        crankPIDController.setIMaxAccum(1, 0);
        crankPIDController.setIZone(0, 0);
        crankPIDController.setFeedbackDevice(crankThruBoreEncoder);
        // TEMPORARY
        crankPIDController.setOutputRange(-1.0, 1.0, 0);

        crankThruBoreEncoder.setPositionConversionFactor(LauncherConfig.CRANK_POSITION_FACTOR);
        crankThruBoreEncoder.setInverted(true);
        crankThruBoreEncoder.setAverageDepth(16);
        crankThruBoreEncoder.setZeroOffset(LauncherConfig.CRANK_ANGLE_OFFSET);
        crankThruBoreEncoder.setVelocityConversionFactor(LauncherConfig.CRANK_POSITION_FACTOR);

        crankMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        crankMotor.burnFlash();

        // Grab the initial crank angles
        crankAngleValues = LauncherConfig.SPEAKER_CRANK_ANGLE;
        // Initialize the crank angles with the initial values
        initCrankAngleOffsets();

        // USE THIS IF THE BLUE SIDE IS DIFFERENT FOR SOME REASON
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent()) {
        //     if (alliance.get() == DriverStation.Alliance.Blue){
        //         crankIncrementer = 2.5;
        //     }
        // }

        SmartDashboard.putNumber("launcher/manualDistanceToTarget", 20);
        SmartDashboard.putNumber("launcher/manualLeftRollerSpeed", 0);
        SmartDashboard.putNumber("launcher/manualRightRollerSpeed", 0);
        SmartDashboard.putNumber("launcher/manualCrankAngle", 90);
    }

    public void readPeriodicInputs() {
        leftMotorClosedLoopErrorSignal.refresh();
        leftMotorSpeedSignal.refresh();
        leftMotorCurrentSignal.refresh();
        leftMotorClosedLoopReferenceSignal.refresh();
        rightMotorClosedLoopErrorSignal.refresh();
        rightMotorSpeedSignal.refresh();
        rightMotorCurrentSignal.refresh();
        rightMotorClosedLoopReferenceSignal.refresh();

        inputs.leftMotorClosedLoopError = leftMotorClosedLoopErrorSignal.asSupplier().get() * 60.0;
        inputs.leftMotorSpeed = leftMotorSpeedSignal.asSupplier().get() * 60.0;
        inputs.leftMotorCurrent = leftMotorCurrentSignal.asSupplier().get();
        inputs.leftMotorClosedLoopReference = leftMotorClosedLoopReferenceSignal.asSupplier().get();

        inputs.rightMotorClosedLoopError = rightMotorClosedLoopErrorSignal.asSupplier().get() * 60.0;
        inputs.rightMotorSpeed = rightMotorSpeedSignal.asSupplier().get() * 60.0;
        inputs.rightMotorCurrent = rightMotorCurrentSignal.asSupplier().get();
        inputs.rightMotorClosedLoopReference = rightMotorClosedLoopReferenceSignal.asSupplier().get();

        inputs.crankAngle = crankThruBoreEncoder.getPosition();
        inputs.crankAngleError = outputs.crankAngleTarget - inputs.crankAngle;

        inputs.currentDistanceToTarget = PoseEstimator.getInstance().getDistanceToTarget();
        inputs.futureDistanceToTarget = PoseEstimator.getInstance().getFutureDistanceToTarget();
//        inputs.distanceForCrossShot = PoseEstimator.getInstance().getFutureDistanceForCrossShot();
        inputs.distanceForCrossShot = PoseEstimator.getInstance().getDistanceForCrossShot();

        inputs.distanceForSourceShot1 = PoseEstimator.getInstance().getDistanceForSourceShot1();
        inputs.distanceForSourceShot2 = PoseEstimator.getInstance().getDistanceForSourceShot2();

        inputs.speakerCrankAngles = crankAngleValues;

        inputs.crankAngleSensorFault = checkCrankAngleSensorFaulted();
    }

    public void processLoop() {
        setManualDistanceToTarget();
        LauncherSystemState newState;
        newState = switch (currentState) {
            case SPINUP -> handleSpinup();
            case READY_TO_FIRE -> handleFire();
            case IDLE -> handleIdle();
        };
        if (newState != currentState) {
            currentState = newState;
        }
    }

    public void writePeriodicOutputs() {
        try {
            calculateLauncherTargets();
        } catch (Exception e) {}
        setManualTargets();
        switch (currentState) {
            case READY_TO_FIRE:
            case SPINUP:
                leftRollerMotor.setControl(rollerSpeedRequest.withVelocity(outputs.leftRollerSpeedTarget / 60));
                rightRollerMotor.setControl(rollerSpeedRequest.withVelocity(outputs.rightRollerSpeedTarget / 60));
                crankPIDController.setReference(outputs.crankAngleTarget, ControlType.kPosition);
                break;

            default:
                leftRollerMotor.stopMotor();
                rightRollerMotor.stopMotor();
                crankPIDController.setReference(outputs.crankAngleTarget, ControlType.kPosition);
                break;

        }
        if (inputs.crankAngleSensorFault){
            crankMotor.stopMotor();
        }
    }

    @Override
    public void stop() {
        leftRollerMotor.stopMotor();
        rightRollerMotor.stopMotor();
        crankMotor.stopMotor();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("trimOffset", crankIncrementer);
    }

    private LauncherSystemState handleIdle() {
        Vision.getInstance().disableLights();
        if (wantedState.equals(LauncherWantedState.FIRE)) {
            return LauncherSystemState.SPINUP;
        } else if (wantedState.equals(LauncherWantedState.SPINUP)) {
            return LauncherSystemState.SPINUP;
        } else {
            return LauncherSystemState.IDLE;
        }
    }

    private boolean checkTeleopTolerance() {
        if(Drivetrain.getInstance().isMoving()) {
            return checkTeleopSpeedTolerance() && checkTeleopCrankToleranceMoving();
        } else {
            return checkTeleopSpeedTolerance() && checkTeleopCrankTolerance();
        }
    }
    private boolean checkTeleopSpeedTolerance() {
        return Math.abs(inputs.leftMotorClosedLoopError) <= LauncherConfig.LEFT_ROLLER_SPEED_TOLERANCE
                && Math.abs(inputs.rightMotorClosedLoopError) <= LauncherConfig.RIGHT_ROLLER_SPEED_TOLERANCE;
    }

    private boolean checkTeleopCrankTolerance() {
        return Math.abs(inputs.crankAngleError) <= LauncherConfig.CRANK_ANGLE_TOLERANCE;
    }
    private boolean checkTeleopCrankToleranceMoving() {
        return Math.abs(inputs.crankAngleError) <= LauncherConfig.CRANK_ANGLE_TOLERANCE_MOVING;
    }

    private boolean checkAutonTolerance() {
        return Math.abs(inputs.leftMotorClosedLoopError) <= LauncherConfig.AUTON_LEFT_ROLLERSPEED_TOLERANCE
                && Math.abs(inputs.rightMotorClosedLoopError) <= LauncherConfig.AUTON_RIGHT_ROLLERSPEED_TOLERANCE
                && Math.abs(inputs.crankAngleError) <= LauncherConfig.CRANK_ANGLE_TOLERANCE;
    }

    private boolean isReadyToFire() {

        if (DriverStation.isAutonomous() && autonQuickshotEnabled) {
            return checkAutonTolerance() /*&& Drivetrain.getInstance().isAimed()*/;

        } else if (shotType.equals(ShotType.SPEAKER)) {
            return checkTeleopTolerance() && Drivetrain.getInstance().isAimed();

        } else if(shotType.equals(ShotType.SUBWOOFER) || shotType.equals(ShotType.AMP)) {
            return checkTeleopTolerance();
        } else {
            return checkTeleopTolerance() && Drivetrain.getInstance().isAimed() && !Drivetrain.getInstance().isMoving();
        }
    }

    private LauncherSystemState handleSpinup() {
        // Need to verify this??
        Led.getInstance().setWantedState(LedWantedState.LOADED_SOLID);
        if (isReadyToFire()) {
            return LauncherSystemState.READY_TO_FIRE;
        } else if (wantedState.equals(LauncherWantedState.IDLE)) {
            return LauncherSystemState.IDLE;
        } else {
            return LauncherSystemState.SPINUP;
        }
    }

    private LauncherSystemState handleFire() {
        // Need to verify this??
        Led.getInstance().setWantedState(LedWantedState.READY_TO_FIRE);
        // take average left and right motor closed loop errors
        double medianAverageError = filter
                .calculate(inputs.leftMotorClosedLoopError + inputs.rightMotorClosedLoopError / 2d);
        if (wantedState.equals(LauncherWantedState.IDLE)) {
            return LauncherSystemState.IDLE;
        } else if (medianAverageError >= LauncherConfig.NOTE_SHOT_ERROR_SPIKE) {
            filter.reset();
            // return LauncherSystemState.IDLE;
        } else if (!isReadyToFire()) {
            // Go BACK to spinup if you aren't ready to fire
            return LauncherSystemState.SPINUP;
        }
        return LauncherSystemState.READY_TO_FIRE;
    }

    private void calculateLauncherTargets() {
        switch (shotType) {
            case AMP:
                outputs.leftRollerSpeedTarget = LauncherConfig.AMP_LEFT_ROLLER_SPEED;
                outputs.rightRollerSpeedTarget = LauncherConfig.AMP_RIGHT_ROLLER_SPEED;
                outputs.crankAngleTarget = LauncherConfig.AMP_CRANK_ANGLE;
                break;
            case SPEAKER:
                outputs.leftRollerSpeedTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.SPEAKER_DISTANCE_BREAKPOINTS,
                        LauncherConfig.SPEAKER_LEFT_ROLLER_SPEED,
                        inputs.futureDistanceToTarget);
                outputs.rightRollerSpeedTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.SPEAKER_DISTANCE_BREAKPOINTS,
                        LauncherConfig.SPEAKER_RIGHT_ROLLER_SPEED,
                        inputs.futureDistanceToTarget);
                outputs.crankAngleTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.SPEAKER_DISTANCE_BREAKPOINTS,
                        crankAngleValues,
                        inputs.futureDistanceToTarget);
                break;
            case SUBWOOFER:
                outputs.leftRollerSpeedTarget = LauncherConfig.SUBWOOFER_LEFT_ROLLER_SPEED;
                outputs.rightRollerSpeedTarget = LauncherConfig.SUBWOOFER_RIGHT_ROLLER_SPEED;
                outputs.crankAngleTarget = getBreakpointAngle(0);
                break;
            case MANUAL:
                outputs.leftRollerSpeedTarget = manualLeftRollerSpeed;
                outputs.rightRollerSpeedTarget = manualRightRollerSpeed;
                outputs.crankAngleTarget = manualCrankAngle;
                break;
            case YEET:
                outputs.leftRollerSpeedTarget = LauncherConfig.YEET_LEFT_ROLLER_SPEED;
                outputs.rightRollerSpeedTarget = LauncherConfig.YEET_RIGHT_ROLLER_SPEED;
                outputs.crankAngleTarget = LauncherConfig.YEET_CRANK_ANGLE;
                break;
            case QUICKSHOT:
                outputs.leftRollerSpeedTarget = LauncherConfig.QUICKSHOT_LEFT_ROLLER_SPEED;
                outputs.rightRollerSpeedTarget = LauncherConfig.QUICKSHOT_RIGHT_ROLLER_SPEED;
                outputs.crankAngleTarget = LauncherConfig.QUICKSHOT_CRANK_ANGLE;
                break;
            case MOONSHOT:
                outputs.leftRollerSpeedTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.CROSS_SHOT_DISTANCE_BREAKPOINTS,
                        LauncherConfig.CROSS_SHOT_LEFT_ROLLER_SPEEDS,
                        inputs.distanceForCrossShot);
                outputs.rightRollerSpeedTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.CROSS_SHOT_DISTANCE_BREAKPOINTS,
                        LauncherConfig.CROSS_SHOT_RIGHT_ROLLER_SPEEDS,
                        inputs.distanceForCrossShot);
                outputs.crankAngleTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.CROSS_SHOT_DISTANCE_BREAKPOINTS,
                        LauncherConfig.CROSS_SHOT_CRANK_ANGLE,
                        inputs.distanceForCrossShot);
                break;
            case SOURCE_SHOT_1:
                outputs.leftRollerSpeedTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.CROSS_SHOT_DISTANCE_BREAKPOINTS,
                        LauncherConfig.CROSS_SHOT_LEFT_ROLLER_SPEEDS,
                        inputs.distanceForSourceShot1
                );
                outputs.rightRollerSpeedTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.CROSS_SHOT_DISTANCE_BREAKPOINTS,
                        LauncherConfig.CROSS_SHOT_RIGHT_ROLLER_SPEEDS,
                        inputs.distanceForSourceShot1
                );
                outputs.crankAngleTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.CROSS_SHOT_DISTANCE_BREAKPOINTS,
                        LauncherConfig.CROSS_SHOT_CRANK_ANGLE,
                        inputs.distanceForSourceShot1
                );
                break;
            case SOURCE_SHOT_2:
                outputs.leftRollerSpeedTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.CROSS_SHOT_DISTANCE_BREAKPOINTS,
                        LauncherConfig.CROSS_SHOT_LEFT_ROLLER_SPEEDS,
                        inputs.distanceForSourceShot2
                );
                outputs.rightRollerSpeedTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.CROSS_SHOT_DISTANCE_BREAKPOINTS,
                        LauncherConfig.CROSS_SHOT_RIGHT_ROLLER_SPEEDS,
                        inputs.distanceForSourceShot2
                );
                outputs.crankAngleTarget = OneDimensionalLookup.interpLinear(
                        LauncherConfig.CROSS_SHOT_DISTANCE_BREAKPOINTS,
                        LauncherConfig.CROSS_SHOT_CRANK_ANGLE,
                        inputs.distanceForSourceShot2
                );

                break;
            default:
                break;

        }
        if (currentState.equals(LauncherSystemState.IDLE)) {
            outputs.leftRollerSpeedTarget = LauncherConfig.IDLE_LEFT_ROLLER_SPEED;
            outputs.rightRollerSpeedTarget = LauncherConfig.IDLE_RIGHT_ROLLER_SPEED;
            outputs.crankAngleTarget = LauncherConfig.IDLE_CRANK_ANGLE;
        }
    }

    public void setManualDistanceToTarget() {
        manualDistanceToTarget = SmartDashboard.getNumber("launcher/manualDistanceToTarget", 20);
    }

    public void setManualTargets() {
        manualLeftRollerSpeed = SmartDashboard.getNumber("launcher/manualLeftRollerSpeed", 0);
        manualRightRollerSpeed = SmartDashboard.getNumber("launcher/manualRightRollerSpeed", 0);
        manualCrankAngle = SmartDashboard.getNumber("launcher/manualCrankAngle", 90);
    }

    public void setWantedState(LauncherWantedState state) {
        if (state != null) {
            wantedState = state;
        }
    }

    public void setShotType(ShotType type) {
        shotType = type;
    }

    public LauncherSystemState getSystemState() {
        return currentState;
    }

    public void setAutonQuickshotTrue() {
        autonQuickshotEnabled = true;
    }

    public void setAutonQuickshotFalse() {
        autonQuickshotEnabled = false;
    }

    public double estimateNoteVelocity() {
        final double efficiencyCoeffecient = 1;
        double avgRpm = (outputs.leftRollerSpeedTarget + outputs.rightRollerSpeedTarget) / 2;
        double avgSurfaceSpeedMps = avgRpm * 3 * Math.PI / 60;
        avgSurfaceSpeedMps = Units.inchesToMeters(avgSurfaceSpeedMps);
        return avgSurfaceSpeedMps * efficiencyCoeffecient;
    }

    private void updateCrankAngles(double incrementAmount) {
        if (crankIncrementer >= -12 && crankIncrementer <= 12) {
            crankIncrementer += incrementAmount;

            double[] newCrankAngles = LauncherConfig.SPEAKER_CRANK_ANGLE.clone();
            for (int i = 0; i < LauncherConfig.CRANK_ANGLE_TRIM_OFFSETS.length; i++) {
                newCrankAngles[i] += LauncherConfig.CRANK_ANGLE_TRIM_OFFSETS[i] * crankIncrementer;
            }
            crankAngleValues = newCrankAngles;
        }
    }

    public void incrementCrankAngleOffsets() {
        updateCrankAngles(0.5);
    }

    public void decrementCrankAngleOffsets() {
        updateCrankAngles(-0.5);
    }

    public void initCrankAngleOffsets() {
        double[] newCrankAngles = LauncherConfig.SPEAKER_CRANK_ANGLE.clone();
        for (int i = 0; i < LauncherConfig.CRANK_ANGLE_TRIM_OFFSETS.length; i++) {
            newCrankAngles[i] += LauncherConfig.CRANK_ANGLE_TRIM_OFFSETS[i] * crankIncrementer;
        }
        crankAngleValues = newCrankAngles;
    }

    public boolean isReady() {
        return currentState.equals(LauncherSystemState.READY_TO_FIRE);
    }

    public double getCrankAngle() {
        return inputs.crankAngle;
    }

    public ShotType getShotType() {
        return shotType;
    }

    public double getAvgRollerVelocityError() {
        return (inputs.leftMotorClosedLoopError + inputs.rightMotorClosedLoopError) / 2d;
    }

    public double getBreakpointAngle(int index) {
        if (index < 0 || index >= LauncherConfig.SPEAKER_CRANK_ANGLE.length) {
            return 0;
        }
        return crankAngleValues[index];
    }

    public double getCrankAngleError() {
        return inputs.crankAngleError;
    }

    private boolean checkCrankAngleSensorFaulted(){
        return inputs.crankAngle < 1.0d;
    }

    public boolean getLauncherFault(){
        return inputs.crankAngleSensorFault;
    }
}
