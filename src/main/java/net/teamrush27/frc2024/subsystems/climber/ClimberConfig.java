package net.teamrush27.frc2024.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import net.teamrush27.frc2024.Robot;
import net.teamrush27.frc2024.util.RobotHelper.RobotType;

public class ClimberConfig {

    public static final int CLIMBER_MOTOR_ID = 40;
    public static final int CLIMBER_RATCHET_SERVO_PORT = 0;
    public static final double CLIMBER_RATIO = (280d/9) * (16d / 17);
    public static final int CLIMBER_CURRENT_LIMIT = 80;
    public static final double CLIMBER_KP = 0.2;
    public static final double CLIMBER_KI = 0;
    public static final double CLIMBER_KD = 0;
    public static final int  CLIMBER_FORWARD_LIMIT = 117;
    public static final int CLIMBER_REVERSE_LIMIT = 2;

    public static final int CLIMBER_HOME_CURRENT =  18;

    public static final double SERVO_LATCH_POSITION = 0;
    public static final double SERVO_UNLATCH_POSITION = 1;
    public static final double CLIMBER_DEPLOY_POSITION = 116;

    public static final double CLIMBER_RETRACT1_POSITION = 3;

    public static final double CLIMB_POSITION_TOLERANCE = 5;

    public static CANSparkFlex configureMotor(int deviceId, MotorType motorType) {
        CANSparkFlex motor = new CANSparkFlex(deviceId, motorType);
        motor.setSmartCurrentLimit(CLIMBER_CURRENT_LIMIT);
        motor.setInverted(true);
        motor.setOpenLoopRampRate(0.1);
        motor.setClosedLoopRampRate(0.1);
        motor.setIdleMode(IdleMode.kBrake);
        motor.getPIDController().setOutputRange(-1, 1);

        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, CLIMBER_FORWARD_LIMIT);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, CLIMBER_REVERSE_LIMIT);

        motor.getPIDController().setP(CLIMBER_KP);
        motor.getPIDController().setI(CLIMBER_KI);
        motor.getPIDController().setD(CLIMBER_KD);

        motor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        motor.burnFlash();
        return motor;
    }
}
