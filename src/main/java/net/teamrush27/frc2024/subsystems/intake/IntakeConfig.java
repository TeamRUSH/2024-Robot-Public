// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.teamrush27.frc2024.subsystems.intake;

import com.revrobotics.*;
import net.teamrush27.frc2024.Robot;
import net.teamrush27.frc2024.util.RobotHelper.RobotType;

/** Add your docs here. */
public class IntakeConfig {

    public static final int ROLLER_MOTOR_ID = 25;
    public static final int DEPLOY_MOTOR_ID = 26;
    public static final int DEPLOY_MOTOR_CURRENT_LIMIT = 40;
    public static final int INTAKE_ROLLER_CURRENT_LIMIT = 80;
    public static final double INTAKE_POS_ERROR_TOLERANCE_DEG = 5;
    public static final double ROLLER_DUTY_IDLE = 0;
    public static final double ROLLER_DUTY_DEPLOYED = 1.0;
    public static final double ROLLER_DUTY_EXHAUST = -1.0;
    public static final double ROLLER_DUTY_RETRACT = -0.3;
    public static double DEPLOY_ANGLE_STOW;
    public static double DEPLOY_ANGLE_OUT;
    public static final double LAMPREY_SENSOR_SCALE = 360 / 3.3;
    public static final double INTAKE_DEPLOY_REDUCTION = 360d / 48;//38.5;

    // Deploy feedback parameters
    public static final double DEPLOY_KP = 0.015;

    public static void setRobot(){
        if (Robot.robotType.equals(RobotType.PRACTICE)){
            DEPLOY_ANGLE_STOW = 334;
            DEPLOY_ANGLE_OUT = 227;
        } else if(Robot.robotType.equals(RobotType.COMPETITION)){
            DEPLOY_ANGLE_STOW = 403; // 395
            DEPLOY_ANGLE_OUT = 303;
        }
    }

    public static CANSparkFlex configureDeployMotor(int canId) {
        CANSparkFlex deployMotor = new CANSparkFlex(canId, CANSparkLowLevel.MotorType.kBrushless);
        deployMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        deployMotor.setSmartCurrentLimit(IntakeConfig.DEPLOY_MOTOR_CURRENT_LIMIT);
        deployMotor.setInverted(true);

        deployMotor.getPIDController().setFeedbackDevice(deployMotor.getEncoder());

        deployMotor.getPIDController().setP(IntakeConfig.DEPLOY_KP, 0);

        return deployMotor;
    }

    public static SparkAnalogSensor configAnalogEncoder(CANSparkFlex deployMotor) {
        SparkAnalogSensor lamprey = deployMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        lamprey.setPositionConversionFactor(LAMPREY_SENSOR_SCALE);
        lamprey.setInverted(true);

        deployMotor.getPIDController().setFeedbackDevice(lamprey);
        return lamprey;
    }

    public static RelativeEncoder configRelativeEncoder(CANSparkFlex deployMotor) {
        RelativeEncoder encoder = deployMotor.getEncoder();
        encoder.setPositionConversionFactor(INTAKE_DEPLOY_REDUCTION);

        return encoder;
    }

    public static CANSparkFlex configureRollerMotor(int canId) {
        CANSparkFlex rollerMotor = new CANSparkFlex(canId, CANSparkLowLevel.MotorType.kBrushless);

        rollerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        rollerMotor.setInverted(true);
        rollerMotor.setSmartCurrentLimit(IntakeConfig.INTAKE_ROLLER_CURRENT_LIMIT);

        rollerMotor.burnFlash();
        return rollerMotor;
    }
}
