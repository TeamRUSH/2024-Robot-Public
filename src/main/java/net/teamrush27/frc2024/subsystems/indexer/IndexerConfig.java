package net.teamrush27.frc2024.subsystems.indexer;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;

public class IndexerConfig {
    public static final int INDEX_MOTOR_ID = 30;
    public static final int INDEXER_INTAKE_SENSOR_PORT = 0;
    public static final int INDEXER_FRONT_SENSOR_PORT = 1;
    public static final int INDEXER_BACK_SENSOR_PORT = 2;
    public static final double INDEXER_IDLE_RPM = 0;
    public static final double INDEXER_INTAKE_RPM = 3000;
    public static final double INDEXER_INDEX_RPM = 2600;
    public static final double INDEXER_LOADING_RPM = -1500;
    public static final double INDEXER_FEED_RPM = 6000;
    public static final double INDEXER_EXHAUST_DUTY = -1.0;

    public static final double SENSOR_DEBOUNCE_TIME = 0.05;
    public static final double FRONT_RISE_DEBOUNCE_TIME = 0.02;
    public static final double FRONT_FALL_DEBOUNCE_TIME = 0.02;

    public static final double INDEX_MOTOR_REDUCTION = 1;
    public static final double INDEX_MOTOR_KF = 0.00016;
    public static final double INDEX_MOTOR_KP = 0.0001;
    public static final double INDEX_MOTOR_KI = 0;
    public static final double INDEX_MOTOR_KD = 0;
    public static final int INDEX_MOTOR_CURRENT_LIMIT = 40;
    public static final CANSparkBase.IdleMode INDEX_MOTOR_IDLE_MODE = CANSparkBase.IdleMode.kCoast;


    public static CANSparkFlex initConfiguredMotor(int deviceId, MotorType motorType) {
        CANSparkFlex motor = new CANSparkFlex(deviceId, motorType);

        motor.setSmartCurrentLimit(INDEX_MOTOR_CURRENT_LIMIT);
        motor.setIdleMode(INDEX_MOTOR_IDLE_MODE);
        motor.setInverted(true);
        motor.getEncoder().setVelocityConversionFactor(INDEX_MOTOR_REDUCTION);

        motor.getPIDController().setFF(INDEX_MOTOR_KF);
        motor.getPIDController().setP(INDEX_MOTOR_KP, 0);
        motor.getPIDController().setI(INDEX_MOTOR_KI, 0);
        motor.getPIDController().setD(INDEX_MOTOR_KD, 0);
        motor.getPIDController().setOutputRange(-1, 1, 0);
        motor.burnFlash();
        return motor;
    }
}
