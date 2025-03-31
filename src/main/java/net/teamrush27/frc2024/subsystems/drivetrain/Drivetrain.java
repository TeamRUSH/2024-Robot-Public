package net.teamrush27.frc2024.subsystems.drivetrain;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import net.teamrush27.frc2024.Constants;
import net.teamrush27.frc2024.Robot;
import net.teamrush27.frc2024.controlboard.ControlBoard;
import net.teamrush27.frc2024.controlboard.IDriverControlBoard;
import net.teamrush27.frc2024.generated.TunerConstants;
import net.teamrush27.frc2024.generated.TunerConstants_2024Comp;
import net.teamrush27.frc2024.subsystems.launcher.Launcher;
import net.teamrush27.frc2024.subsystems.poseEstimator.PoseEstimator;
import net.teamrush27.frc2024.subsystems.vision.Vision;
import net.teamrush27.frc2024.util.RobotHelper.RobotType;
import net.teamrush27.frc2024.util.RushMath;


public class Drivetrain extends Subsystem implements edu.wpi.first.wpilibj2.command.Subsystem {
    @IgnoreLogged
    static Drivetrain instance;
    RushSwerveDrivetrain drivetrain;
    SwerveRequest wantedRequest;
    IDriverControlBoard controlBoard;
    @Log
    double yawError;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }

        return instance;
    }

    private final DrivetrainInputs inputs;
    private final DrivetrainOutputs outputs;
    private MedianFilter txFilter = new MedianFilter(5);
    @Log
    private WantedState wantedState = WantedState.FIELD_CENTRIC;
    
    // boolean isAimed = false;
    private Alliance alliance = Alliance.Blue;
    private Rotation2d operatorPerspective;


    private double VELOCITY_SCALER = 1d;

    private final SwerveRequest.FieldCentricFacingAngle autoAimRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.MAX_LINEAR_VELOCITY_MPS * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagic);

    {
        autoAimRequest.HeadingController = new PhoenixPIDController(10.0, 0, 0);
        autoAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private ProfiledPIDController autoAimController = DrivetrainConfig.autoAimController;

    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagic);

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagic);

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagic);

    private final SwerveRequest.ApplyChassisSpeeds auton = new SwerveRequest.ApplyChassisSpeeds();

    public Drivetrain() {
        if (Robot.robotType.equals(RobotType.COMPETITION)) {
            this.drivetrain = TunerConstants_2024Comp.generate();
        } else {
            this.drivetrain = TunerConstants.generate();
        }
        this.controlBoard = ControlBoard.getInstance().getDriverControlBoard();

        Pathfinding.setPathfinder(new LocalADStar());

        inputs = new DrivetrainInputs();
        outputs = new DrivetrainOutputs();

        // Set the method that will be used to get rotation overrides
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setTrajectorySpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5, 0.0, 0.1), // Translation PID constants
                        new PIDConstants(5, 0.0, 0/*.15*/), // Rotation PID constants
                        Constants.MAX_LINEAR_VELOCITY_MPS, // Max module speed, in m/s
                        Constants.DRIVETRAIN_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(false, true) // Default path replanning config. See the API for the options
                                                          // here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        autoAimController.enableContinuousInput(-Math.PI, Math.PI);
        autoAimController.setTolerance(Units.degreesToRadians(DrivetrainConfig.AUTO_AIM_ANGLE_TOLERANCE_DEG));
        autoAimController.setIZone(DrivetrainConfig.AUTO_AIM_CONTROLLER_IZONE);

        fieldCentric.ForwardReference = SwerveRequest.ForwardReference.OperatorPerspective;
    }

    // enumerated type representing the desired state of the drivetrain, which is
    // also the internal state
    public enum WantedState {
        AUTON,
        FIELD_CENTRIC,
        ROBOT_CENTRIC,
        BRAKE,
        SPEAKER_ALIGN,
        AUTON_SPEAKER_TRACKING,
        FACING_ANGLE,
        NOTE_TRACKING
    }

    @Override
    public void readPeriodicInputs() {

        double controllerX = controlBoard.getThrottle();
        double controllerY = controlBoard.getStrafe();
        double controllerRot = controlBoard.getRotation();
        double controllerMagnitude = RushMath.pythagorean(controllerX, controllerY);

        inputs.magnitudeCommand = DrivetrainConfig.applyJoystickFunction(
                Math.min(controllerMagnitude, 1),
                2
        );

        // create unit vectors
        double unitX = controllerMagnitude == 0 ? 0 : controllerX / controllerMagnitude;
        double unitY = controllerMagnitude == 0 ? 0 : controllerY / controllerMagnitude;

        inputs.xCommand = unitX * inputs.magnitudeCommand * Constants.MAX_LINEAR_VELOCITY_MPS * VELOCITY_SCALER;
        inputs.yCommand = unitY * inputs.magnitudeCommand * Constants.MAX_LINEAR_VELOCITY_MPS * VELOCITY_SCALER;
        inputs.omegaCommand = DrivetrainConfig.applyJoystickFunction(controllerRot, 3) * Constants.MAX_ROTATIONAL_VELOCITY_RPS * VELOCITY_SCALER;

        inputs.moduleStates = drivetrain.getState().ModuleStates;
        outputs.targetStates = drivetrain.getState().ModuleTargets;
        inputs.yaw = drivetrain.getPigeon2().getRotation2d();
        inputs.yawRate = Rotation2d.fromDegrees(drivetrain.getPigeon2().getRate());

        for (int i = 0; i < 4; i++) {
            inputs.moduleDriveVelocitiesMps[i] = drivetrain.getModule(i).getDriveMotor().getVelocity().getValue();
            inputs.moduleDriveCurrentsMps[i] = drivetrain.getModule(i).getDriveMotor().getStatorCurrent().getValue();
            inputs.moduleDriveSupplyCurrents[i] = drivetrain.getModule(i).getDriveMotor().getSupplyCurrent().getValue();
            inputs.moduleDriveTemps[i] = drivetrain.getModule(i).getDriveMotor().getDeviceTemp().getValue();
            inputs.moduleSteeringPositions[i] = drivetrain.getModule(i).getSteerMotor().getPosition().getValue();
            inputs.moduleSteerErrors[i] = drivetrain.getModule(i).getSteerMotor().getClosedLoopError().getValue();
        }

        inputs.robotPose = drivetrain.getState().Pose;
        inputs.angleToTarget = PoseEstimator.getInstance().getFutureRotationToTarget().getRadians();

        inputs.angleToCrossShot = switch(Launcher.getInstance().getShotType()) {
            case SOURCE_SHOT_1 -> PoseEstimator.getInstance().getRotationForSourceShot1().getRadians();
            case SOURCE_SHOT_2 -> PoseEstimator.getInstance().getRotationForSourceShot2().getRadians();
            default -> PoseEstimator.getInstance().getRotationForCrossShot().getRadians();
        };

        inputs.robotAngleToField = PoseEstimator.getInstance().getDrivebaseAngleToField().getRadians();
        inputs.yawError = autoAimController.getPositionError();

        if(DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

        if(alliance.equals(Alliance.Red)) {
            operatorPerspective = Rotation2d.fromDegrees(180);
        } else {
            operatorPerspective = Rotation2d.fromDegrees(0);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper loops) {
        loops.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // resetOdometry();
                drivetrain.getDaqThread().setThreadPriority(99);
                // drivetrain.getDaqThread().start();
            }

            @Override
            public void onLoop(double timestamp) {
                drivetrain.setOperatorPerspectiveForward(operatorPerspective);
                switch (wantedState) {
                    case FIELD_CENTRIC -> {
                        wantedRequest = fieldCentric
                                .withVelocityX(inputs.xCommand)
                                .withVelocityY(inputs.yCommand)
                                .withRotationalRate(inputs.omegaCommand);
                    }

                    case ROBOT_CENTRIC -> {
                        wantedRequest = robotCentric
                                .withVelocityX(inputs.xCommand)
                                .withVelocityY(inputs.yCommand)
                                .withRotationalRate(inputs.omegaCommand);
                    }

                    case AUTON, AUTON_SPEAKER_TRACKING -> {
                        wantedRequest = auton.withSpeeds(outputs.trajectorySpeeds);
                    }

                    case BRAKE -> {
                        wantedRequest = brakeRequest;
                    }

                    case SPEAKER_ALIGN -> {
                        if (Math.abs(controlBoard.getRotation()) < 0.12) {
                            wantedRequest = fieldCentric
                                    .withVelocityX(inputs.xCommand)
                                    .withVelocityY(inputs.yCommand)
                                    .withRotationalRate(autoAimController.calculate(
                                            inputs.robotAngleToField,
                                            inputs.angleToTarget));

                        } else {
                            wantedRequest = fieldCentric
                                    .withVelocityX(inputs.xCommand)
                                    .withVelocityY(inputs.yCommand)
                                    .withRotationalRate(inputs.omegaCommand);
                        }
                    }
                    case FACING_ANGLE -> {
                        if (Math.abs(controlBoard.getRotation()) < 0.12) {
                            wantedRequest = fieldCentric
                                    .withVelocityX(inputs.xCommand)
                                    .withVelocityY(inputs.yCommand)
                                    .withRotationalRate(autoAimController.calculate(
                                            inputs.robotAngleToField,
                                            inputs.angleToCrossShot));
                        } else {
                            wantedRequest = fieldCentric
                                    .withVelocityX(inputs.xCommand)
                                    .withVelocityY(inputs.yCommand)
                                    .withRotationalRate(inputs.omegaCommand);
                        }
                    }
                    case NOTE_TRACKING -> {
                        Vision vision = Vision.getInstance();
                        double angleToNote = 0;
                        if (vision.getNoteTx().isPresent()) {
                            angleToNote = Units.degreesToRadians(vision.getNoteTx().get());
                        }
                        wantedRequest = robotCentric
                                .withVelocityX(inputs.xCommand)
                                .withVelocityY(inputs.yCommand)
                                .withRotationalRate(
                                        autoAimController.calculate(
                                                inputs.robotAngleToField,
                                                inputs.robotAngleToField + angleToNote
                                        ) + inputs.omegaCommand
                                );
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                wantedRequest = new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds());
            }

        });
    }

    @Override
    public void writePeriodicOutputs() {
        drivetrain.setControl(wantedRequest);
        yawError = Math.abs(inputs.angleToTarget - inputs.robotPose.getRotation().getRadians());
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry(boolean isDisabled) {

    }

    public RushSwerveDrivetrain getSwerveDrivetrainInstance() {
        return drivetrain;
    }

    public void setWantedState(WantedState newState) {
        wantedState = newState;
    }

    public void setTrajectorySpeeds(ChassisSpeeds speeds) {
        outputs.trajectorySpeeds = speeds;
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        drivetrain.seedFieldRelative(pose);
    }

    public ChassisSpeeds getSpeeds() {
        return drivetrain.getSwerveDriveKinematics().toChassisSpeeds(drivetrain.getState().ModuleStates);
    }

    @Log
    public boolean isAimed() {
        return isAimed(DrivetrainConfig.AUTO_AIM_ANGLE_TOLERANCE_DEG);
    }
    public boolean isAimed(double degrees) {
        double aimError = 0;
        if(wantedState.equals(WantedState.SPEAKER_ALIGN)) {
            aimError = inputs.robotAngleToField - inputs.angleToTarget;
        } else if(wantedState.equals(WantedState.FACING_ANGLE)) {
            aimError = inputs.robotAngleToField - inputs.angleToCrossShot;
        }

        if (aimError < Units.degreesToRadians(degrees) /*|| (RushMath.pythagorean(inputs.xCommand, inputs.yCommand) < 0.5 && inputs.omegaCommand < 0.1)*/){
            return true;
        }

        return (wantedState.equals(WantedState.AUTON_SPEAKER_TRACKING) && autoAimController.atSetpoint());
    }

    @Log
    public boolean isMoving() {
        return Math.hypot(drivetrain.getState().speeds.vxMetersPerSecond, drivetrain.getState().speeds.vyMetersPerSecond) > 2.0;
    }

    public Optional<Rotation2d> getRotationTargetOverride(){
        // Some condition that should decide if we want to override rotation
        if(this.wantedState.equals(WantedState.AUTON_SPEAKER_TRACKING)) {
            // Return an optional containing the rotation override (this should be a field relative rotation)
            return Optional.of(Rotation2d.fromRadians(inputs.angleToTarget));
        } else {
            // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }
    }

    public void setIntakeSpeed(){
        VELOCITY_SCALER = 0.7;
    }

    public void resetDrivetrainSpeed(){
        VELOCITY_SCALER = 1d;
    }

    public double getYawError() {
        return inputs.yawError;
    }

    public void resetTxFilter() {
        txFilter.reset();
    }

}
