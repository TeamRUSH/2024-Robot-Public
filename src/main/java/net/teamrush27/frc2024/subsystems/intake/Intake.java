package net.teamrush27.frc2024.subsystems.intake;


import com.revrobotics.*;
import com.revrobotics.SparkAnalogSensor.Mode;

import edu.wpi.first.math.filter.MedianFilter;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import net.teamrush27.frc2024.util.NoteSubsystem;

public class Intake extends NoteSubsystem {

    @IgnoreLogged
    static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private final CANSparkFlex deployMotor;
    private final CANSparkFlex rollerMotor;
    private final SparkAnalogSensor lamprey;
    private final RelativeEncoder relativeEncoder;

    private final MedianFilter lampreyFilter = new MedianFilter(5);


    private final IntakeInputs inputs = new IntakeInputs();
    private final IntakeOutputs intakeOutputs = new IntakeOutputs();
    @Log
    private IntakeWantedState intakeWantedState = IntakeWantedState.IDLE;
    @Log
    private IntakeSystemState systemState = IntakeSystemState.INIT;


    private Intake() {
        IntakeConfig.setRobot();
        rollerMotor = IntakeConfig.configureRollerMotor(IntakeConfig.ROLLER_MOTOR_ID);
        deployMotor = IntakeConfig.configureDeployMotor(IntakeConfig.DEPLOY_MOTOR_ID);

        lamprey = IntakeConfig.configAnalogEncoder(deployMotor);
        relativeEncoder = IntakeConfig.configRelativeEncoder(deployMotor);

        deployMotor.burnFlash();

        relativeEncoder.setPosition(lamprey.getPosition());
    }

    @Override
    public void readPeriodicInputs() {

        inputs.lampreyPosition = lamprey.getPosition();
        inputs.filteredLampreyPosition = lampreyFilter.calculate(inputs.lampreyPosition);
        inputs.relativePosition = relativeEncoder.getPosition();
        inputs.deployMotorCurrent = deployMotor.getOutputCurrent();
        inputs.rollerMotorCurrent = rollerMotor.getOutputCurrent();
        inputs.deployMotorAppliedOutput = deployMotor.getAppliedOutput();
    }

    @Override
    public void processLoop() {
        systemState = systemState.handle(intakeWantedState, inputs);
        systemState.setOutputs(intakeOutputs);
    }

    @Override
    public void writePeriodicOutputs() {
        if(systemState.equals(IntakeSystemState.DEPLOY) || systemState.equals(IntakeSystemState.RETRACT) || systemState.equals(IntakeSystemState.CLIMB)){
            deployMotor.getPIDController().setReference(intakeOutputs.targetAngle, CANSparkBase.ControlType.kPosition);
        } else {
            deployMotor.stopMotor();
        }
        rollerMotor.set(intakeOutputs.rollerDuty);
        outputTelemetry();
    }

    @Override
    public void stop() {
        deployMotor.stopMotor();
        rollerMotor.stopMotor();
    }

    @Override
    public void outputTelemetry() {
    }

    public void setWantedState(IntakeWantedState newState) {
        if(newState != null && !intakeWantedState.equals(IntakeWantedState.CLIMB)) {
            this.intakeWantedState = newState;
        }
    }

    public void setSystemState(IntakeSystemState intakeSystemState) {
        this.systemState = intakeSystemState;
    }

    public IntakeSystemState getSystemState() {
        return systemState;
    }
}
