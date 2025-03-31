package net.teamrush27.frc2024.subsystems.supervisor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import monologue.Annotations;
import net.teamrush27.frc2024.subsystems.drivetrain.Drivetrain;
import net.teamrush27.frc2024.subsystems.indexer.IndexerSystemState;
import net.teamrush27.frc2024.subsystems.indexer.IndexerWantedState;
import net.teamrush27.frc2024.subsystems.intake.IntakeWantedState;
import net.teamrush27.frc2024.subsystems.launcher.Launcher;
import net.teamrush27.frc2024.subsystems.launcher.LauncherSystemState;
import net.teamrush27.frc2024.subsystems.leds.LedWantedState;
import net.teamrush27.frc2024.subsystems.poseEstimator.PoseEstimator;

public enum SupervisorSystemState {
    IDLE() {
        @Override
        public SupervisorSystemState handle(SupervisorWantedState wantedState, SupervisorInputs inputs) {
            this.intakeWantedState = IntakeWantedState.IDLE;
            this.indexerWantedState = IndexerWantedState.IDLE;
            this.ledWantedState = LedWantedState.IDLE;

            return switch (wantedState) {
                case IDLE, FIRE -> inputs.indexerState.equals(IndexerSystemState.LOADED) ? LOADED : IDLE;
                case INTAKE -> inputs.indexerState.equals(IndexerSystemState.LOADED) ? LOADED : INTAKE;
                case EXHAUST -> EXHAUST;
                case PASSTHROUGH -> PASSTHROUGH;
            };
        }
    },
    INTAKE() {
        @Override
        public SupervisorSystemState handle(SupervisorWantedState wantedState, SupervisorInputs inputs) {
            this.intakeWantedState = IntakeWantedState.INTAKE;
            this.indexerWantedState = IndexerWantedState.INDEX;
            this.ledWantedState = LedWantedState.INTAKE_NOTE;
            if(inputs.indexerState.equals(IndexerSystemState.INDEX)) {
                this.ledWantedState = LedWantedState.HAS_NOTE;
            }

            return switch (wantedState) {
                case IDLE, FIRE -> inputs.indexerState.equals(IndexerSystemState.LOADING) ? LOADING : IDLE;
                case INTAKE -> inputs.indexerState.equals(IndexerSystemState.LOADING) ? LOADING :  INTAKE;
                case EXHAUST -> EXHAUST;
                case PASSTHROUGH -> PASSTHROUGH;
            };
        }
    },
    LOADING() {
        @Override
        public SupervisorSystemState handle(SupervisorWantedState wantedState, SupervisorInputs inputs) {
            this.intakeWantedState = IntakeWantedState.IDLE;
            this.indexerWantedState = IndexerWantedState.INDEX;
            this.ledWantedState = LedWantedState.HAS_NOTE;


            return switch(wantedState) {
                case EXHAUST -> EXHAUST;
                default -> inputs.indexerState.equals(IndexerSystemState.LOADED) ? LOADED : LOADING;
            };
        }
    },

    LOADED() {
        @Override
        public SupervisorSystemState handle(SupervisorWantedState wantedState, SupervisorInputs inputs) {
            this.intakeWantedState = IntakeWantedState.IDLE;
            this.indexerWantedState = IndexerWantedState.IDLE;
            this.ledWantedState = LedWantedState.LOADED_STROBE;

            if(DriverStation.isAutonomous()) {
                this.intakeWantedState = IntakeWantedState.INTAKE;
            }

            if(inputs.timeInState > 1) {
                this.ledWantedState = LedWantedState.LOADED_SOLID;
            }
            return switch (wantedState) {
                case IDLE, INTAKE -> LOADED;
                case EXHAUST -> EXHAUST;
                case FIRE -> {
                    this.dumpShotInfo(inputs);
                    yield inputs.launcherState.equals(LauncherSystemState.READY_TO_FIRE) ? FIRE : LOADED;
                }
                case PASSTHROUGH -> PASSTHROUGH;
            };
        }
    },

    FIRE() {
        @Override
        public SupervisorSystemState handle(SupervisorWantedState wantedState, SupervisorInputs inputs) {
            this.intakeWantedState = IntakeWantedState.IDLE;
            this.indexerWantedState = IndexerWantedState.FEED;
            this.ledWantedState = LedWantedState.RAINBOW;

            return switch (wantedState) {
                case IDLE -> IDLE;
                case FIRE -> FIRE;
                case INTAKE -> INTAKE;
                case EXHAUST -> EXHAUST;
                case PASSTHROUGH -> PASSTHROUGH;
            };
        }
    },

    EXHAUST() {

        @Override
        public SupervisorSystemState handle(SupervisorWantedState wantedState, SupervisorInputs inputs) {
            this.intakeWantedState = IntakeWantedState.EXHAUST;
            this.indexerWantedState = IndexerWantedState.EXHAUST;
            this.ledWantedState = LedWantedState.EXHAUST;

            return switch (wantedState) {
                case IDLE, INTAKE, FIRE -> IDLE;
                case EXHAUST -> EXHAUST;
                case PASSTHROUGH -> PASSTHROUGH;
            };
        }
    }, 
    
    PASSTHROUGH() {

        @Override
        public SupervisorSystemState handle(SupervisorWantedState wantedState, SupervisorInputs input){

            this.intakeWantedState = IntakeWantedState.INTAKE;
            this.indexerWantedState = IndexerWantedState.FEED;
            this.ledWantedState = LedWantedState.RAINBOW;

            return switch (wantedState){
                case IDLE -> IDLE;
                case INTAKE -> INTAKE;
                case FIRE -> FIRE;
                case EXHAUST -> EXHAUST;
                case PASSTHROUGH -> PASSTHROUGH;
            };
        }
    };

    IntakeWantedState intakeWantedState;
    IndexerWantedState indexerWantedState;
    LedWantedState ledWantedState;

    public static boolean intakeOut = false;

    public static boolean passthrough = false;

    public static boolean overrideShot = false;

    public abstract SupervisorSystemState handle(SupervisorWantedState wantedState, SupervisorInputs inputs);

    public void setOutputs(SupervisorInputs inputs, SupervisorOutputs outputs) {
        outputs.intakeWantedState = this.intakeWantedState;
        outputs.indexerWantedState = this.indexerWantedState;
        outputs.ledWantedState = this.ledWantedState;
        if(intakeOut) {
            outputs.intakeWantedState = IntakeWantedState.INTAKE;
            outputs.indexerWantedState = IndexerWantedState.INDEX;
            if(this.indexerWantedState == IndexerWantedState.FEED) {
                outputs.indexerWantedState = IndexerWantedState.FEED;
            }
        }
        if(!overrideShot && intakeOut) {
            outputs.indexerWantedState = IndexerWantedState.INDEX;
        }
        if((passthrough && inputs.launcherState == LauncherSystemState.READY_TO_FIRE) || overrideShot) {
            outputs.indexerWantedState = IndexerWantedState.FEED;
        }
    }

    void dumpShotInfo(SupervisorInputs inputs) {
        PoseEstimator pe = PoseEstimator.getInstance();
        DriverStation.reportWarning(
            String.format(
                "Distance: %.1f, Crank: %.1f, CrankError: %.1f, YawError: %.1f, isAimed: %b, avgSpeedError: %.1f, SubwooferAngle: %.1f",
                pe.getFutureDistanceToTarget(),
                Launcher.getInstance().getCrankAngle(),
                Launcher.getInstance().getCrankAngleError(),
                Units.radiansToDegrees(Drivetrain.getInstance().getYawError()),
                Drivetrain.getInstance().isAimed(),
                Launcher.getInstance().getAvgRollerVelocityError(),
                Launcher.getInstance().getBreakpointAngle(0)
            ),
            false
        );
    }
}
