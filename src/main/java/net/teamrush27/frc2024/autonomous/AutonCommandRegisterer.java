package net.teamrush27.frc2024.autonomous;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.*;
import net.teamrush27.frc2024.autonomous.commands.*;
import net.teamrush27.frc2024.subsystems.drivetrain.Drivetrain;
import net.teamrush27.frc2024.subsystems.drivetrain.Drivetrain.WantedState;
import net.teamrush27.frc2024.subsystems.indexer.Indexer;
import net.teamrush27.frc2024.subsystems.indexer.IndexerSystemState;
import net.teamrush27.frc2024.subsystems.intake.Intake;
import net.teamrush27.frc2024.subsystems.intake.IntakeSystemState;
import net.teamrush27.frc2024.subsystems.launcher.Launcher;
import net.teamrush27.frc2024.subsystems.launcher.LauncherWantedState;
import net.teamrush27.frc2024.subsystems.poseEstimator.PoseEstimator;
import net.teamrush27.frc2024.subsystems.supervisor.Supervisor;
import net.teamrush27.frc2024.subsystems.supervisor.SupervisorSystemState;
import net.teamrush27.frc2024.subsystems.supervisor.SupervisorWantedState;
import net.teamrush27.frc2024.subsystems.vision.Vision;
import net.teamrush27.frc2024.util.LimelightHelpers;

public class AutonCommandRegisterer {
    static Drivetrain drivetrain = Drivetrain.getInstance();
    static Supervisor noteSupervisor = Supervisor.getInstance();
    static Launcher launcher = Launcher.getInstance();
    static Indexer indexer = Indexer.getInstance();
    static Intake intake = Intake.getInstance();
    static PoseEstimator poseEstimator = PoseEstimator.getInstance();
    static Vision vision = Vision.getInstance();

    static AutoDriver autoDriver = new AutoDriver();


    public static void registerNamedCommands(){
            NamedCommands.registerCommand("intakeDeploy", new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.INTAKE)));
            NamedCommands.registerCommand("intakeDeploySequential", new GrabNote());

            NamedCommands.registerCommand("launcherFireSequential", new ShootNote());
            // NamedCommands.registerCommand("launcherFireSequential", new WaitCommand(1));

//            NamedCommands.registerCommand("enablePassthrough", new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.PASSTHROUGH)));
//            NamedCommands.registerCommand("disablePassthrough", new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.IDLE)));

            NamedCommands.registerCommand("exhaust", new SequentialCommandGroup(
                new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.EXHAUST)),
                new WaitCommand(.3),
                new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.IDLE))
            ));
            
            NamedCommands.registerCommand("trackSpeakerEnable", new InstantCommand(() -> drivetrain.setWantedState(WantedState.AUTON_SPEAKER_TRACKING)));
            NamedCommands.registerCommand("trackSpeakerDisable", new InstantCommand(() -> drivetrain.setWantedState(WantedState.AUTON)));

            NamedCommands.registerCommand("toleranceEnable", new InstantCommand(() -> launcher.setAutonQuickshotFalse()));

            NamedCommands.registerCommand("autonInit", new ParallelCommandGroup(
                new InstantCommand(() -> launcher.setShotType(Launcher.ShotType.SPEAKER)),
                new InstantCommand(() -> launcher.setWantedState(LauncherWantedState.SPINUP)),
                new InstantCommand(() -> noteSupervisor.setCurrentState(SupervisorSystemState.LOADED)),
                new InstantCommand(()-> intake.setSystemState(IntakeSystemState.IDLE)),
                new InstantCommand(() -> indexer.setSystemState(IndexerSystemState.LOADED))
            ));

            NamedCommands.registerCommand("forceOdometryUpdate", new InstantCommand((poseEstimator::resetDrivetrainOdometry)));

            NamedCommands.registerCommand("intakeThenShoot", new SequentialCommandGroup(
                                                                new GrabNote().withTimeout(15),
                                                                new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.FIRE)),
                                                                new WaitCommand(.3),
                                                                new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.IDLE))
                                                            ));

            NamedCommands.registerCommand("launcherFire", new SequentialCommandGroup(
                                                                new ShootNote(),
                                                                // new WaitCommand(.3),
                                                                new WaitCommand(.4),
                                                                new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.IDLE))
                                                            ));

            NamedCommands.registerCommand("quickshot", new SequentialCommandGroup(
                                                                new Quickshot(),
                                                                new WaitCommand(.3),
                                                                new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.IDLE))
                                                            ));

            NamedCommands.registerCommand("quickshotStart", new SequentialCommandGroup(
                                                                new InstantCommand(() -> launcher.setShotType(Launcher.ShotType.QUICKSHOT)),
                                                                new InstantCommand(() -> launcher.setWantedState(LauncherWantedState.SPINUP)),
                                                                new InstantCommand(() -> noteSupervisor.setCurrentState(SupervisorSystemState.LOADED)),
                                                                new InstantCommand(()-> intake.setSystemState(IntakeSystemState.IDLE)),
                                                                new InstantCommand(() -> indexer.setSystemState(IndexerSystemState.LOADED)),
                                                                new Quickshot2(),
                                                                new InstantCommand(() -> launcher.setShotType(Launcher.ShotType.SPEAKER)),
                                                                new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.IDLE))
                                                            ));

            NamedCommands.registerCommand("enableTeleopTracking",new InstantCommand(() -> drivetrain.setWantedState(Drivetrain.WantedState.SPEAKER_ALIGN)));

            NamedCommands.registerCommand("enableLeaveIntakeOut", new InstantCommand(() -> noteSupervisor.leaveIntakeOut(true)));
            NamedCommands.registerCommand("disableLeaveIntakeOut", new InstantCommand(() -> noteSupervisor.leaveIntakeOut(false)));

            NamedCommands.registerCommand(
                    "enablePassthrough",
                    new InstantCommand(() -> noteSupervisor.setPassthrough(true))
            );

        NamedCommands.registerCommand(
                "disablePassthrough",
                new ParallelCommandGroup(
                        new InstantCommand(() -> noteSupervisor.setPassthrough(false)),
                        new InstantCommand(() -> noteSupervisor.setCurrentState(SupervisorSystemState.IDLE))
                )
        );

        NamedCommands.registerCommand(
                "enableLeaveIntakeOut",
                new InstantCommand(() -> noteSupervisor.leaveIntakeOut(true))
        );

        NamedCommands.registerCommand(
                "disableLeaveIntakeOut",
                new InstantCommand(() -> noteSupervisor.leaveIntakeOut(false))
        );

        NamedCommands.registerCommand(
                "forceFire",
                new SequentialCommandGroup(
                        new InstantCommand(() -> noteSupervisor.setShotOverride(true)),
                        new WaitCommand(1),
                        new InstantCommand(() -> noteSupervisor.setShotOverride(false))
                )
        );

        NamedCommands.registerCommand(
                "enableShotOverride",
                new InstantCommand(() -> noteSupervisor.setShotOverride(true))
        );
        NamedCommands.registerCommand(
                "disableShotOverride",
                new InstantCommand(() -> noteSupervisor.setShotOverride(false))
        );

        NamedCommands.registerCommand(
                "waitUntilSpunUp",
                new WaitUntilSpunUp()
        );

        NamedCommands.registerCommand(
                "stationaryAimAtSpeaker",
                new AimAtSpeaker()
        );

        NamedCommands.registerCommand(
                "huntNote",
                new ConditionalCommand(
//                        AutoBuilder.pathfindToPose(
//                                poseEstimator.getNotePose(),
//                                new PathConstraints(1.0, 1.0, 1, 1)
//                        ),
                        Commands.print(""),
                        Commands.print("THERE WERE NO NOTES!!!!!!!!!!"),
                        () -> vision.seesNote()
                )
        );

        NamedCommands.registerCommand(
                "incrementByHalf",
                new InstantCommand(launcher::incrementCrankAngleOffsets)
        );

        NamedCommands.registerCommand(
                "decrementByHalf",
                new InstantCommand(launcher::decrementCrankAngleOffsets)
        );






        // sequential command to turn on tracking, fire when in tolerance (low rotation speed and close to 0 error on spkr), turn off tracking
            // NamedCommands.registerCommand("trackedShot", new SequentialCommandGroup(
            //                                                     new Quickshot(),
            //                                                     new WaitCommand(.3),
            //                                                     new InstantCommand(() -> noteSupervisor.setWantedState(SupervisorWantedState.IDLE))
            //                                                 ));
    }
}


