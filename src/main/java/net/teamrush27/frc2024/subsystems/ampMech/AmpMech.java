package net.teamrush27.frc2024.subsystems.ampMech;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import net.teamrush27.frc2024.util.NoteSubsystem;

public class AmpMech extends NoteSubsystem {

    @IgnoreLogged
    static AmpMech instance;

    public static AmpMech getInstance() {
        if (instance == null) {
            instance = new AmpMech();
        }
        return instance;
    }
    private final CANSparkMax deployMotor;

    public enum AmpMechWantedState {
        RETRACT,
        DEPLOY
    }
    @Log
    AmpMechWantedState wantedState = AmpMechWantedState.RETRACT;

    private enum SystemState{
        RETRACTED,
        RETRACTING,
        DEPLOYING,
        DEPLOYED,
        HOMING
    }
    
    @Log
    SystemState systemState = SystemState.RETRACTED;

    AmpMechIO.Inputs inputs = new AmpMechIO.Inputs();
    AmpMechIO.Outputs outputs = new AmpMechIO.Outputs();

    private AmpMech(){
        deployMotor = AmpMechConfig.initAmpMechMotor();
    }
    @Override
    public void readPeriodicInputs() {
        inputs.motorDutyActual = deployMotor.getAppliedOutput();
        inputs.current = deployMotor.getOutputCurrent();
        inputs.position = deployMotor.getEncoder().getPosition();
        inputs.positionError = Math.abs(outputs.positionCommand - inputs.position);
    }

    @Override
    public void writePeriodicOutputs() {
        deployMotor.getPIDController().setReference(outputs.positionCommand, CANSparkBase.ControlType.kPosition);
    }

    @Override
    public void processLoop() {
        SystemState newState;
        newState = switch(systemState){
            case RETRACTED -> handleRetracted();
            case RETRACTING -> handleRetracting();
            case DEPLOYING -> handleDeploying();
            case DEPLOYED -> handleDeployed();
            case HOMING -> handleHoming();
        };
        systemState = newState;
    }

    @Override
    public void stop() {
        deployMotor.stopMotor();
    }

    @Override
    public void outputTelemetry() {
    }

    private SystemState handleRetracted(){
        outputs.positionCommand = AmpMechConfig.RETRACTED_POSITION;
        return switch(wantedState){
            case RETRACT -> SystemState.RETRACTED;
            case DEPLOY -> SystemState.DEPLOYING;
        };
    }

    private SystemState handleRetracting(){
        outputs.positionCommand = AmpMechConfig.RETRACTED_POSITION;
        return switch(wantedState){
            case RETRACT -> inputs.positionError < AmpMechConfig.POSITION_TOLERANCE ?
                    SystemState.RETRACTED : SystemState.RETRACTING;
            case DEPLOY -> SystemState.DEPLOYING;
        };
    }

    private SystemState handleDeploying(){
        outputs.positionCommand = AmpMechConfig.DEPLOYED_POSITION;
        return switch(wantedState){
            case RETRACT -> SystemState.RETRACTING;
            case DEPLOY -> inputs.positionError < AmpMechConfig.POSITION_TOLERANCE ?
                    SystemState.DEPLOYED : SystemState.DEPLOYING;
        };
    }

    private SystemState handleDeployed(){
        outputs.positionCommand = AmpMechConfig.DEPLOYED_POSITION;
        return switch(wantedState){
            case RETRACT -> SystemState.RETRACTING;
            case DEPLOY -> SystemState.DEPLOYED;
        };
    }

    private SystemState handleHoming(){
        return SystemState.HOMING;

    }

    public void setWantedState(AmpMechWantedState state){
        this.wantedState = state;
    }
}

