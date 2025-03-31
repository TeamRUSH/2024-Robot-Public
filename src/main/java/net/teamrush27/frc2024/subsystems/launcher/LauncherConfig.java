// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.teamrush27.frc2024.subsystems.launcher;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team254.lib.util.InterpolatingTreeMap;

import net.teamrush27.frc2024.Robot;
import net.teamrush27.frc2024.util.RobotHelper.RobotType;
import net.teamrush27.frc2024.Robot;
import net.teamrush27.frc2024.util.RobotHelper.RobotType;


/** Add your docs here. */
public class LauncherConfig {

    // Roller motor setup, dual Krakens on Canivore bus
    public static final int LEFT_ROLLER_CAN_ID = 35;
    public static final int RIGHT_ROLLER_CAN_ID = 36;
    public static final String ROLLER_CANBUS = "CANFD";

    private static final MotorOutputConfigs leftRollerMotorOutputConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withPeakForwardDutyCycle(1.0)
        .withPeakReverseDutyCycle(0);
    private static final FeedbackConfigs leftRollerFeedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio(0.636363);

    private static final MotorOutputConfigs rightRollerMotorOutputConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withPeakForwardDutyCycle(1.0)
        .withPeakReverseDutyCycle(0);
    private static final FeedbackConfigs rightRollerFeedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio(1.0);

    private static final CurrentLimitsConfigs rollerCurrentLimitsConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

    private static final Slot0Configs rollerSlot0Configs = new Slot0Configs()
        .withKV(0.0845)
        .withKP(0.3)
        .withKI(0.04)
        .withKD(0);

    private static final Slot0Configs rightRollerSlot0Configs = new Slot0Configs()
        .withKV(0.13)
        .withKP(0.5)
        .withKI(0.04)
        .withKD(0);

    public static final TalonFXConfiguration leftRollerConfig = new TalonFXConfiguration()
    .withCurrentLimits(rollerCurrentLimitsConfigs)
    .withFeedback(leftRollerFeedbackConfigs)
    .withMotorOutput(leftRollerMotorOutputConfigs)
    .withSlot0(rollerSlot0Configs);

    public static final TalonFXConfiguration rightRollerConfig = new TalonFXConfiguration()
    .withCurrentLimits(rollerCurrentLimitsConfigs)
    .withFeedback(rightRollerFeedbackConfigs)
    .withMotorOutput(rightRollerMotorOutputConfigs)
    .withSlot0(rightRollerSlot0Configs);



    // Launcher crank, Neo Vortex with Spark Flex, must be on RIO canbus
    public static final int CRANK_CAN_ID = 37;
    public static final float CRANK_POSITION_UPPER_LIMIT = 188;
    public static final float CRANK_POSITION_LOWER_LIMIT = 20;
    public static final double CRANK_POSITION_FACTOR = 360;
    public static final double CRANK_KP = 0.05;

    public static double CRANK_ANGLE_OFFSET;


    // Shot Parameters
    public static double[] SPEAKER_DISTANCE_BREAKPOINTS;
    public static double[] SPEAKER_LEFT_ROLLER_SPEED;
    public static double[] SPEAKER_RIGHT_ROLLER_SPEED;
    public static double[] SPEAKER_CRANK_ANGLE;

    public static double QUICKSHOT_LEFT_ROLLER_SPEED = 1500;
    public static double QUICKSHOT_RIGHT_ROLLER_SPEED = 1000;
    public static double QUICKSHOT_CRANK_ANGLE = 154;

    public static double AMP_LEFT_ROLLER_SPEED = 1500;
    public static double AMP_RIGHT_ROLLER_SPEED = 1000;
    public static double AMP_CRANK_ANGLE = 129;

//    public static double SUBWOOFER_LEFT_ROLLER_SPEED = 4500;
//    public static double SUBWOOFER_RIGHT_ROLLER_SPEED = 3000;
//    public static double SUBWOOFER_CRANK_ANGLE = 147;

    public static double SUBWOOFER_LEFT_ROLLER_SPEED = 1500*0.75;
    public static double SUBWOOFER_RIGHT_ROLLER_SPEED = 1000*0.75;
    public static double SUBWOOFER_CRANK_ANGLE = 120;

    public static final double YEET_LEFT_ROLLER_SPEED = 5000;
    public static final double YEET_RIGHT_ROLLER_SPEED = 3500;
    public static final double YEET_CRANK_ANGLE = 85;

    public static final double IDLE_LEFT_ROLLER_SPEED = 0;
    public static final double IDLE_RIGHT_ROLLER_SPEED = 0;

    // public static final double IDLE_LEFT_ROLLER_SPEED = 3000;
    // public static final double IDLE_RIGHT_ROLLER_SPEED = 1000;
    public static final double IDLE_CRANK_ANGLE = AMP_CRANK_ANGLE;

    public static final double LEFT_ROLLER_SPEED_TOLERANCE = 500; // 200
    public static final double RIGHT_ROLLER_SPEED_TOLERANCE = 500; //200
    public static final double CRANK_ANGLE_TOLERANCE = 3;
    public static final double CRANK_ANGLE_TOLERANCE_MOVING = 6;

    public static final double AUTON_LEFT_ROLLERSPEED_TOLERANCE = 3000; // TODO
    public static final double AUTON_RIGHT_ROLLERSPEED_TOLERANCE = 3000; // TODO
    public static final double NOTE_SHOT_ERROR_SPIKE = 300; // TODO
    public static final double[] CROSS_SHOT_DISTANCE_BREAKPOINTS = {22, 25, 30, 33.5};

    // public static final double[] CROSS_SHOT_LEFT_ROLLER_SPEEDS = {3250, 3700, 4000, 4200};
    public static final double[] CROSS_SHOT_LEFT_ROLLER_SPEEDS = {3250, 3700, 4350, 4000};
    // public static final double[] CROSS_SHOT_RIGHT_ROLLER_SPEEDS = {2000, 2400, 2650, 2850};
    public static final double[] CROSS_SHOT_RIGHT_ROLLER_SPEEDS = {2000, 2500, 3000, 2800};

    public static final double[] CROSS_SHOT_CRANK_ANGLE = {116, 115, 105, 100}; //116, 115, 107, 107
    // public static final double[] CROSS_SHOT_CRANK_ANGLE = {116, 113, 105, 105};

    public static double[] CRANK_ANGLE_TRIM_OFFSETS = { 3.35, 1.65, 1.64, 1.95, 2.5, 3.24, 3.95 };

    public static void setRobot(){
        if (Robot.robotType.equals(RobotType.PRACTICE)){
            CRANK_ANGLE_OFFSET = 271.8-60;

            //After CMP

            SPEAKER_DISTANCE_BREAKPOINTS = new double[]{4.5, 7.5, 10, 12.5, 15, 17.5, 20, 25};
            SPEAKER_LEFT_ROLLER_SPEED = new double[]{4000, 4500, 4700, 4900, 5100, 6000, 6100, 6200};
            SPEAKER_RIGHT_ROLLER_SPEED = new double[]{2600, 3100, 3250, 3375, 3500, 4000, 4000, 4200};
            SPEAKER_CRANK_ANGLE = new double[]{147, 105, 85, 70, 64, 45, 34, 26};
            CRANK_ANGLE_TRIM_OFFSETS = new double[]{ 3.35, 1.65, 1.64, 1.8, 1.95, 2.5, 3.24, 3.95 };



            SUBWOOFER_LEFT_ROLLER_SPEED = SPEAKER_LEFT_ROLLER_SPEED[0];
            SUBWOOFER_RIGHT_ROLLER_SPEED = SPEAKER_RIGHT_ROLLER_SPEED[0];
            SUBWOOFER_CRANK_ANGLE = SPEAKER_CRANK_ANGLE[0];

            AMP_LEFT_ROLLER_SPEED = 1800;
            AMP_RIGHT_ROLLER_SPEED = 1800;
            AMP_CRANK_ANGLE = 130;

        } else if(Robot.robotType.equals(RobotType.COMPETITION)){
            CRANK_ANGLE_OFFSET = 276.76 + 60;

            SPEAKER_DISTANCE_BREAKPOINTS = new double[]{4.5, 7.5, 10, 12.5, 15, 17.5, 20, 25};

            SPEAKER_LEFT_ROLLER_SPEED = new double[]{4500, 4500, 4700, 4900, 5100, 6000, 6100, 6200}; // 4000
            SPEAKER_RIGHT_ROLLER_SPEED = new double[]{3100, 3100, 3250, 3375, 3500, 4000, 4000, 4200}; // 2600

            SPEAKER_CRANK_ANGLE = new double[]{147, 105, 84, 70, 61, 45, 34, 26};

            CRANK_ANGLE_TRIM_OFFSETS = new double[]{ 3.35, 1.65, 1.64, 1.8, 1.95, 2.5, 3.24, 3.95 };



            SUBWOOFER_LEFT_ROLLER_SPEED = SPEAKER_LEFT_ROLLER_SPEED[0];
            SUBWOOFER_RIGHT_ROLLER_SPEED = SPEAKER_RIGHT_ROLLER_SPEED[0];
            SUBWOOFER_CRANK_ANGLE = SPEAKER_CRANK_ANGLE[0];

            AMP_LEFT_ROLLER_SPEED = 1800;
            AMP_RIGHT_ROLLER_SPEED = 1800;
            // AMP_CRANK_ANGLE = 132;

            // msc
            AMP_CRANK_ANGLE = 130;
        }



    }

}
