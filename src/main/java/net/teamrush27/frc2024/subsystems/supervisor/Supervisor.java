package net.teamrush27.frc2024.subsystems.supervisor;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.drivers.Subsystem;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;

import edu.wpi.first.wpilibj.DriverStation;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import net.teamrush27.frc2024.subsystems.ampMech.AmpMech;
import net.teamrush27.frc2024.subsystems.indexer.Indexer;
import net.teamrush27.frc2024.subsystems.intake.Intake;
import net.teamrush27.frc2024.subsystems.launcher.Launcher;
import net.teamrush27.frc2024.subsystems.launcher.Launcher.ShotType;
import net.teamrush27.frc2024.subsystems.leds.Led;
import net.teamrush27.frc2024.subsystems.leds.LedWantedState;
import net.teamrush27.frc2024.subsystems.vision.Vision;
import net.teamrush27.frc2024.util.NoteSubsystem;

public class Supervisor extends Subsystem {

    @IgnoreLogged
    static Supervisor instance;
    @IgnoreLogged
    private final Intake intake;
    @IgnoreLogged
    private final Indexer indexer;
    @IgnoreLogged
    private final Launcher launcher;
    @IgnoreLogged
    private final Led led;
    @IgnoreLogged
    private final AmpMech ampMech;
    @IgnoreLogged
    private final List<NoteSubsystem> noteSubsystems = new ArrayList<>();

    @Log
    private SupervisorWantedState wantedState = SupervisorWantedState.IDLE;
    @Log
    private SupervisorSystemState currentState = SupervisorSystemState.IDLE;

    private double mStateStartTime = 0;

    public static Supervisor getInstance() {
        if (instance == null) {
            instance = new Supervisor();
        }

        return instance;
    }

    private Supervisor() {
        intake = Intake.getInstance();
        noteSubsystems.add(intake);

        indexer = Indexer.getInstance();
        noteSubsystems.add(indexer);

        launcher = Launcher.getInstance();
        noteSubsystems.add(launcher);

        ampMech = AmpMech.getInstance();
        noteSubsystems.add(ampMech);

        led = Led.getInstance();
    }

    SupervisorInputs inputs = new SupervisorInputs();
    SupervisorOutputs outputs = new SupervisorOutputs();

    @Override
    public void readPeriodicInputs() {
        inputs.intakeState = intake.getSystemState();
        inputs.indexerState = indexer.getSystemState();
        inputs.launcherState = launcher.getSystemState();
        noteSubsystems.forEach(NoteSubsystem::readPeriodicInputs);
        if (launcher.getLauncherFault() && DriverStation.isDisabled()){
            led.setWantedState(LedWantedState.FAULT);
        } else if (DriverStation.isDisabled()){
            led.setWantedState(LedWantedState.DISABLED);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                SupervisorSystemState lastState = currentState;
                currentState = currentState.handle(wantedState, inputs);

                // Reset state start time if state has changed
                if (!lastState.equals(currentState)) {
                    mStateStartTime = timestamp;
                }
                // update timeInState with new timestamp
                inputs.timeInState = timestamp - mStateStartTime;

                currentState.setOutputs(inputs, outputs);
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void stop() {
        noteSubsystems.forEach(NoteSubsystem::stop);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void writePeriodicOutputs() {
        intake.setWantedState(outputs.intakeWantedState);
        indexer.setWantedState(outputs.indexerWantedState);
        if (launcher.getLauncherFault()){
            led.setWantedState(LedWantedState.FAULT);
        } else {led.setWantedState(outputs.ledWantedState);}
        noteSubsystems.forEach(NoteSubsystem::processLoop);
        noteSubsystems.forEach(NoteSubsystem::writePeriodicOutputs);

    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        noteSubsystems.forEach(NoteSubsystem::outputTelemetry);
    }

    public void setWantedState(SupervisorWantedState wantedState) {
        this.wantedState = wantedState;
    }

    // ONLY USE IN AUTON
    public void setCurrentState(SupervisorSystemState currentState) {
        this.currentState = currentState;
    }

    public void setLauncherShotType(ShotType shotType) {
        launcher.setShotType(shotType);
    }

    public void setAmpMechWantedState(AmpMech.AmpMechWantedState wantedState) {
        ampMech.setWantedState(wantedState);
    }

    public SupervisorSystemState getCurrentState() {
        return this.currentState;
    }

    public SupervisorWantedState getWantedState() {
        return this.wantedState;
    }

    public void leaveIntakeOut(boolean bool) {
        SupervisorSystemState.intakeOut = bool;
    }

    public void setPassthrough(boolean bool) {
        SupervisorSystemState.passthrough = bool;
    }

    public void setShotOverride(boolean bool) {
        SupervisorSystemState.overrideShot = bool;
    }

}
