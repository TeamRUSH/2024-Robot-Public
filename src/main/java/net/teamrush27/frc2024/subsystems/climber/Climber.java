package net.teamrush27.frc2024.subsystems.climber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Servo;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;

public class Climber extends Subsystem{

    @IgnoreLogged
    static Climber instance;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    private final CANSparkFlex climberMotor;
    private final Servo ratchetServo;
    private final SparkLimitSwitch bottomSensor;
    private final SparkLimitSwitch topSensor;
    private final MedianFilter currentFilter = new MedianFilter(5);

    private final ClimberInputs inputs = new ClimberInputs();
    private final ClimberOutputs outputs = new ClimberOutputs();
    @Log
    private ClimberWantedState wantedState = ClimberWantedState.IDLE;
    @Log
    private ClimberSystemState systemState = ClimberSystemState.HOMING;
    private Climber() {
        climberMotor = ClimberConfig.configureMotor(ClimberConfig.CLIMBER_MOTOR_ID, MotorType.kBrushless);
        bottomSensor = climberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        topSensor = climberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        ratchetServo = new Servo(ClimberConfig.CLIMBER_RATCHET_SERVO_PORT);
    }

    @Override
    public void readPeriodicInputs() {
        inputs.bottomSensor = bottomSensor.isPressed();
        inputs.topSensor = topSensor.isPressed();

        inputs.medianMotorCurrent = currentFilter.calculate(climberMotor.getOutputCurrent());
        inputs.motorCurrent = climberMotor.getOutputCurrent();
        inputs.motorSpeed = climberMotor.getEncoder().getVelocity();
        inputs.motorPosition = climberMotor.getEncoder().getPosition();
        inputs.motorOutputVoltage = climberMotor.getOutputCurrent();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {

                // Get old state
                ClimberSystemState oldState = systemState;

                // Get new state
                systemState = systemState.handle(inputs, outputs, wantedState);

                // Zero encoder ONLY on the state transition
                if(systemState.equals(ClimberSystemState.HOME) && oldState.equals(ClimberSystemState.HOMING)) {
                    climberMotor.getEncoder().setPosition(0);
                }

                outputs.motorSetpoint = systemState.getMotorSetpoint();
                outputs.controlType = systemState.getControlType();
                outputs.servoSetpoint = systemState.getServoPosition();
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void writePeriodicOutputs() {
        if(systemState.equals(ClimberSystemState.HOMING)) {
            climberMotor.set(-0.1);
        } else {
            climberMotor.getPIDController().setReference(outputs.motorSetpoint, outputs.controlType);
        }
        ratchetServo.set(outputs.servoSetpoint);
    }

    @Override
    public void stop() {
        climberMotor.stopMotor();
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public void setWantedState(ClimberWantedState newWantedState) {
        wantedState = newWantedState;
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
    }

    public boolean isDeployed() {
        return systemState.equals(ClimberSystemState.DEPLOYED);
    }

}
