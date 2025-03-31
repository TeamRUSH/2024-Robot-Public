package net.teamrush27.frc2024.subsystems.indexer;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import net.teamrush27.frc2024.util.NoteSubsystem;

public class Indexer extends NoteSubsystem {

    @IgnoreLogged
    static Indexer instance;

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    private final CANSparkFlex indexMotor;
    private final DigitalInput intakeSensor = new DigitalInput(IndexerConfig.INDEXER_INTAKE_SENSOR_PORT);
    private final DigitalInput frontSensor = new DigitalInput(IndexerConfig.INDEXER_FRONT_SENSOR_PORT);
    private final DigitalInput backSensor = new DigitalInput(IndexerConfig.INDEXER_BACK_SENSOR_PORT);
    private final Debouncer frontDebouncerRise = new Debouncer(IndexerConfig.FRONT_RISE_DEBOUNCE_TIME,
            Debouncer.DebounceType.kRising);
    private final Debouncer frontDebouncerFall = new Debouncer(IndexerConfig.FRONT_RISE_DEBOUNCE_TIME,
            Debouncer.DebounceType.kFalling);
    private final Debouncer backDebouncer = new Debouncer(IndexerConfig.SENSOR_DEBOUNCE_TIME,
            Debouncer.DebounceType.kBoth);
    private final Debouncer intakeDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    // be a constant
    private final IndexerIO.IndexerInputs inputs = new IndexerIO.IndexerInputs();
    private final IndexerIO.IndexerOutputs outputs = new IndexerIO.IndexerOutputs();

    @Log
    private IndexerWantedState indexerWantedState = IndexerWantedState.IDLE;
    @Log
    private IndexerSystemState systemState = IndexerSystemState.IDLE;

    private Indexer() {
        indexMotor = IndexerConfig.initConfiguredMotor(IndexerConfig.INDEX_MOTOR_ID, MotorType.kBrushless);
    }

    @Override
    public void readPeriodicInputs() {
        inputs.intakeSensorDebounced = intakeDebouncer.calculate(!intakeSensor.get());
        inputs.frontSensorDebounced = frontDebouncerRise.calculate(frontDebouncerFall.calculate(!frontSensor.get()));
        inputs.backSensorDebounced = backDebouncer.calculate(!backSensor.get());
        inputs.motorSpeed = indexMotor.getEncoder().getVelocity();
        inputs.motorTemperature = indexMotor.getMotorTemperature();
        inputs.motorCurrent = indexMotor.getOutputCurrent();
    }

    @Override
    public void processLoop() {
        outputs.indexRPM = systemState.getRpm();
        systemState = systemState.handle(indexerWantedState, inputs);
    }

    @Override
    public void stop() {
        indexMotor.stopMotor();
    }

    @Override
    public void writePeriodicOutputs() {
        if (systemState.equals(IndexerSystemState.EXHAUST)) {
            indexMotor.set(IndexerConfig.INDEXER_EXHAUST_DUTY);
        } else {
            indexMotor.getPIDController().setReference(
                    outputs.indexRPM,
                    CANSparkBase.ControlType.kVelocity);
        }
    }

    @Override
    public void outputTelemetry() {
    }

    public boolean isNotePresent() {
        return inputs.intakeSensorDebounced;
    }

    public void setWantedState(IndexerWantedState state) {
        if(state != null) {
            this.indexerWantedState = state;
        }
    }

    public void setSystemState(IndexerSystemState systemState) {
        this.systemState = systemState;
    }

    public IndexerSystemState getSystemState() {
        return systemState;
    }

}
