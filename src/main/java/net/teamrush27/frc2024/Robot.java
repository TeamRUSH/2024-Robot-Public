// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.teamrush27.frc2024;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.team254.lib.loops.Looper;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import net.teamrush27.frc2024.autonomous.AutonCommandRegisterer;
import net.teamrush27.frc2024.autonomous.commands.AutoDriver;
import net.teamrush27.frc2024.controlboard.ControlBoard;
import net.teamrush27.frc2024.subsystems.ampMech.AmpMech;
import net.teamrush27.frc2024.subsystems.climber.Climber;
import net.teamrush27.frc2024.subsystems.climber.ClimberWantedState;
import net.teamrush27.frc2024.subsystems.drivetrain.Drivetrain;
import net.teamrush27.frc2024.subsystems.indexer.Indexer;
import net.teamrush27.frc2024.subsystems.indexer.IndexerSystemState;
import net.teamrush27.frc2024.subsystems.intake.Intake;
import net.teamrush27.frc2024.subsystems.intake.IntakeSystemState;
import net.teamrush27.frc2024.subsystems.intake.IntakeWantedState;
import net.teamrush27.frc2024.subsystems.launcher.Launcher;
import net.teamrush27.frc2024.subsystems.launcher.LauncherWantedState;
import net.teamrush27.frc2024.subsystems.leds.Led;
import net.teamrush27.frc2024.subsystems.leds.LedWantedState;
import net.teamrush27.frc2024.subsystems.poseEstimator.PoseEstimator;
import net.teamrush27.frc2024.subsystems.poseEstimator.PoseEstimatorConfig;
import net.teamrush27.frc2024.subsystems.supervisor.Supervisor;
import net.teamrush27.frc2024.subsystems.supervisor.SupervisorSystemState;
import net.teamrush27.frc2024.subsystems.supervisor.SupervisorWantedState;
import net.teamrush27.frc2024.subsystems.vision.Vision;
import net.teamrush27.frc2024.util.CrashTracker;
import net.teamrush27.frc2024.util.RobotHelper;
import net.teamrush27.frc2024.util.RobotHelper.RobotType;

public class Robot extends TimedRobot implements Logged {
    private Command m_autonomousCommand;

    @Log
    public static RobotType robotType;

    private RobotContainer m_robotContainer;
    private AutoDriver autodriver;
    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final Looper mEnabledLooper = new Looper(0.02); // TODO: see if 10ms loop time is feasible
    private final Looper mDisabledLooper = new Looper(0.02);

    public enum MatchPhase {
        AUTONOMOUS,
        TELEOP,
        TEST,
        INIT;
    }

    @Log.NT
    public static double matchTime = Timer.getMatchTime();

    @Log.NT
    public static MatchPhase currentPhase = MatchPhase.INIT;

    @Log.NT
    public static double batteryLevel;

    // Declare subsystems
    private PowerDistribution powerDistribution;
    private ControlBoard controlBoard;
    private Drivetrain drivetrain;
    private Supervisor noteSupervisor;
    private Intake intake;
    private Indexer indexer;
    private Launcher launcher;
    private Led led;
    private AmpMech ampMech;
    private Climber climber;
    private PoseEstimator stateEstimator;

    @Override
    public void robotInit() {
        CrashTracker.logRobotInit();
        try {
            robotType = RobotHelper.getRobotType();
            switch (getRuntimeType()) {
                case kRoboRIO, kRoboRIO2: {
                    RobotController.setBrownoutVoltage(6.0);
                    powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
                    break;

                }
                case kSimulation: {
                    break;
                }
            }

            drivetrain = Drivetrain.getInstance();
            noteSupervisor = Supervisor.getInstance();
            intake = Intake.getInstance();
            indexer = Indexer.getInstance();
            launcher = Launcher.getInstance();
            ampMech = AmpMech.getInstance();
            led = Led.getInstance();
            stateEstimator = PoseEstimator.getInstance();
            climber = Climber.getInstance();
            controlBoard = ControlBoard.getInstance();
            m_robotContainer = new RobotContainer();
            autodriver = new AutoDriver();

            mSubsystemManager.setSubsystems(drivetrain, stateEstimator, noteSupervisor, led, climber);
            // This will register enabled loops & all subsystems
            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            // This will JUST register disabled loops
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            // We seperately enable the LEDS on both loops
            led.registerEnabledLoops(mEnabledLooper);
            led.registerEnabledLoops(mDisabledLooper);

            configureBindings();

            AutonCommandRegisterer.registerNamedCommands();
            m_robotContainer.configureAuton();

            SmartDashboard.putData(CommandScheduler.getInstance());
            SmartDashboard.putNumber("Match Time", matchTime);

            mSubsystemManager.stopSubsystems();

            Monologue.setupMonologue(this, "Robot", false, true);
            // DriverStation.startDataLog(DataLogManager.getLog());
            // DataLogManager.logNetworkTables(false);

            addPeriodic(() -> {
                matchTime = Timer.getMatchTime();
                batteryLevel = powerDistribution.getVoltage();
            }, 0.5);


        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // Update matchtime
        Monologue.updateAll();

        SmartDashboard.putNumber("Distance to Speaker: ", stateEstimator.getFutureDistanceToTarget());

    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mSubsystemManager.stopSubsystems();
            mEnabledLooper.stop();
            mDisabledLooper.start();
            Vision.getInstance().disableLights();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        stateEstimator.resetDrivetrainOdometry();

        
        if(m_robotContainer.getAutonomousCommand().getName().contains("SOURCE") && Vision.getInstance().inSourceStartingPosition()){
            Led.getInstance().setWantedState(LedWantedState.RAINBOW);
        }
        else if(m_robotContainer.getAutonomousCommand().getName().contains("Amp") && Vision.getInstance().inAmpStartingPosition()){
            Led.getInstance().setWantedState(LedWantedState.RAINBOW);
        }
        else{

            Led.getInstance().setWantedState(LedWantedState.DISABLED);
        }
        SmartDashboard.putString( "SelectedAuton: ", m_robotContainer.getAutonomousCommand().getName());
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
            m_autonomousCommand = m_robotContainer.getAutonomousCommand();

/*
            m_autonomousCommand = new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new InstantCommand(() -> noteSupervisor.leaveIntakeOut(true)),
                            new InstantCommand(() -> noteSupervisor.setPassthrough(true)),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> launcher.setShotType(Launcher.ShotType.SPEAKER)),
                                    new InstantCommand(() -> launcher.setWantedState(LauncherWantedState.SPINUP)),
                                    new InstantCommand(() -> noteSupervisor.setCurrentState(SupervisorSystemState.LOADED)),
                                    new InstantCommand(()-> intake.setSystemState(IntakeSystemState.IDLE)),
                                    new InstantCommand(() -> indexer.setSystemState(IndexerSystemState.LOADED))
                            )                    ),
                    Commands.waitSeconds(2),
                    AutoBuilder.pathfindToPose(
                            stateEstimator.getNotePose(),
                            new PathConstraints(3.0, 4.0, 5, 5)
                    ),
                    new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.SPEAKER_ALIGN)),
                    Commands.waitSeconds(10)
            );

 */


            drivetrain.setWantedState(Drivetrain.WantedState.AUTON);

            stateEstimator.resetDrivetrainOdometry();
            if (m_autonomousCommand != null) {
                m_autonomousCommand.schedule();
            }
            currentPhase = MatchPhase.AUTONOMOUS;

            mDisabledLooper.stop();
            mSubsystemManager.stopSubsystems();
            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        launcher.setWantedState(LauncherWantedState.IDLE);
        noteSupervisor.setWantedState(SupervisorWantedState.IDLE);
        noteSupervisor.leaveIntakeOut(false);
        noteSupervisor.setPassthrough(false);
        noteSupervisor.setShotOverride(false);
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            if (m_autonomousCommand != null) {
                m_autonomousCommand.cancel();
            }
            PoseEstimatorConfig.NOTE_FLIGHT_TIME_SCALAR = 1;
            currentPhase = MatchPhase.TELEOP;
            drivetrain.setWantedState(Drivetrain.WantedState.FIELD_CENTRIC);
            stateEstimator.resetDrivetrainOdometry();
            mDisabledLooper.stop();
            mSubsystemManager.stopSubsystems();
            mEnabledLooper.start();
            noteSupervisor.setPassthrough(false);
            noteSupervisor.leaveIntakeOut(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        currentPhase = MatchPhase.TEST;
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
        drivetrain.getSwerveDrivetrainInstance().updateSimState(0.020, RobotController.getBatteryVoltage());
    }

    public void configureBindings() {

        /* OPERATOR CONTROLS */
        controlBoard.getIntake()
                .onTrue(new ParallelCommandGroup(new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.INTAKE)), new InstantCommand(()-> drivetrain.setIntakeSpeed())))
                .onFalse(new ParallelCommandGroup(new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.IDLE)), new InstantCommand(()-> drivetrain.resetDrivetrainSpeed())));

        controlBoard.getExhaust()
                .onTrue(new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.EXHAUST)))
                .onFalse(new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.IDLE)));

        controlBoard.getSpinUp()
                .onTrue(new InstantCommand(() -> launcher.setWantedState(LauncherWantedState.SPINUP)));

        controlBoard.getFire()
                .onTrue(new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.FIRE)))
                .onFalse(new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            // Only reset supervisor to intake is not pressed
                            if (!controlBoard.getIntake().getAsBoolean()) {
                                noteSupervisor.setWantedState(SupervisorWantedState.IDLE);
                            }
                        }),
                        new InstantCommand(() -> launcher.setWantedState(LauncherWantedState.IDLE)),
                        new InstantCommand(
                                () -> noteSupervisor.setAmpMechWantedState(AmpMech.AmpMechWantedState.RETRACT))));

        controlBoard.getSubwooferShot()
                .onTrue(new ParallelCommandGroup(
                        new InstantCommand(() -> launcher.setShotType(Launcher.ShotType.SUBWOOFER)),
                        new InstantCommand(
                                () -> noteSupervisor.setAmpMechWantedState(AmpMech.AmpMechWantedState.RETRACT))));

        controlBoard.getAmpShot()
                .onTrue(new ParallelCommandGroup(
                        new InstantCommand(() -> launcher.setShotType(Launcher.ShotType.AMP)),
                        new InstantCommand(
                                () -> noteSupervisor.setAmpMechWantedState(AmpMech.AmpMechWantedState.DEPLOY))));

        /* DRIVER CONTROLS */
        controlBoard.getResetHeading()
                .onTrue(new InstantCommand(stateEstimator::resetDrivetrainOdometry));

        controlBoard.getRobotOriented()
                .onTrue(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.ROBOT_CENTRIC)))
                .onFalse(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.FIELD_CENTRIC)));

        controlBoard.getSpeakerTracking()
                .onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(
                                        () -> drivetrain.setWantedState(Drivetrain.WantedState.SPEAKER_ALIGN)),
                                new InstantCommand(() -> launcher.setShotType(Launcher.ShotType.SPEAKER)),
                                new InstantCommand(() -> noteSupervisor
                                        .setAmpMechWantedState(AmpMech.AmpMechWantedState.RETRACT))))
                .onFalse(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.FIELD_CENTRIC)));

        controlBoard.getNorth()
                .onTrue(
                        new SequentialCommandGroup(
                                new InstantCommand(
                                        () -> noteSupervisor.setAmpMechWantedState(AmpMech.AmpMechWantedState.DEPLOY)),
                                new WaitCommand(0.5),
                                new InstantCommand(() -> climber.setWantedState(ClimberWantedState.DEPLOY))));

        controlBoard.getSouth().and(climber::isDeployed)
                .onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> climber.setWantedState(ClimberWantedState.CLIMB)),
                                new InstantCommand(() -> intake.setWantedState(IntakeWantedState.CLIMB))
                        )
                );

//        controlBoard.getAutoAmp()
//                .whileTrue(
//                        new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.AUTON))
//                                .andThen(autodriver.driveToAmp(DriverStation.getAlliance())))
//                .onFalse(
//                        new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.FIELD_CENTRIC)));

        controlBoard.getCrossFieldShot()
                .onTrue(new ParallelCommandGroup(
                        new InstantCommand(() -> launcher.setShotType(Launcher.ShotType.MOONSHOT)),
                        new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.FACING_ANGLE))))
                .onFalse(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.FIELD_CENTRIC)));

        controlBoard.getSourceShot1()
                        .onTrue(new ParallelCommandGroup(
                                new InstantCommand(() -> launcher.setShotType(Launcher.ShotType.SOURCE_SHOT_1)),
                                new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.FACING_ANGLE))
                        ))
                        .onFalse(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.FIELD_CENTRIC)));

        controlBoard.getSourceShot2()
                        .onTrue(new ParallelCommandGroup(
                                new InstantCommand(() -> launcher.setShotType(Launcher.ShotType.SOURCE_SHOT_2)),
                                new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.FACING_ANGLE))
                        ))
                        .onFalse(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.FIELD_CENTRIC)));

        controlBoard.getNoteTracking()
                    .onTrue(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.NOTE_TRACKING)),
                                    new InstantCommand(drivetrain::resetTxFilter)
                            )
                    )
                    .onFalse(new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.FIELD_CENTRIC)));

        controlBoard.incrementCrankAngle().onTrue(new InstantCommand(()-> launcher.incrementCrankAngleOffsets()));

        controlBoard.decrementCrankAngle().onTrue(new InstantCommand(()-> launcher.decrementCrankAngleOffsets()));

        controlBoard.getLeaveIntakeOut()
                .onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> noteSupervisor.leaveIntakeOut(true)),
                                new InstantCommand(() -> noteSupervisor.setPassthrough(true)),
                                new InstantCommand(() -> launcher.setWantedState(LauncherWantedState.FIRE))
                        )
                )
                .onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> noteSupervisor.leaveIntakeOut(false)),
                                new InstantCommand(() -> noteSupervisor.setPassthrough(false)),
                                new InstantCommand(() -> launcher.setWantedState(LauncherWantedState.IDLE))
                                )
                );
        controlBoard.getShotOverride()
                .onTrue(new InstantCommand(() -> noteSupervisor.setShotOverride(true)))
                .onFalse(new InstantCommand(() -> noteSupervisor.setShotOverride(false)));
    }
}