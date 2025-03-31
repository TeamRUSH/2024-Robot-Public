package net.teamrush27.frc2024.subsystems.ampMech;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;


public class AmpMechConfig {

    public static final int CAN_ID = 45;
    public static final int MAX_CURRENT = 10;
    public static final double KP = 0.1;
    public static final double FWD_DUTY_LIMIT = 0.20; // TODO undo to 0.25
    public static final double REV_DUTY_LIMIT = -0.20; // TODO undo to -0.25

    public static final double DEPLOYED_POSITION = 11d;
    // public static final double DEPLOYED_POSITION = 10.81;
    public static final double RETRACTED_POSITION = 0.0;
    public static final double POSITION_TOLERANCE = 0.25;

    public static CANSparkMax initAmpMechMotor(){
        CANSparkMax motor = new CANSparkMax(CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
        motor.setInverted(true);
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setSmartCurrentLimit(MAX_CURRENT);

        motor.getEncoder().setPosition(0);

        motor.getPIDController().setFF(0, 0);
        motor.getPIDController().setP(KP, 0);
        motor.getPIDController().setI(0, 0);
        motor.getPIDController().setD(0, 0);
        motor.getPIDController().setOutputRange(REV_DUTY_LIMIT, FWD_DUTY_LIMIT, 0);
        motor.burnFlash();
        return motor;
    }
}
