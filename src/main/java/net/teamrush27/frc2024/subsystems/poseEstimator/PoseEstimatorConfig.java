// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.teamrush27.frc2024.subsystems.poseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class PoseEstimatorConfig {

    public static final Matrix<N3, N1> SWERVE_ODOMETRY_STDDEV = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> LAUNCHER_LL_STDDEV = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final Matrix<N3, N1> INTAKE_LL_STDDEV = VecBuilder.fill(0.5, 0.5, 0.5);

    public static final Matrix<N3, N1> STATIONARY_LL_STDDEV = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final Matrix<N3, N1> AUTON_LL_STDDEV = VecBuilder.fill(1.7, 1.7, 9999);
    public static final Matrix<N3, N1> STD_DEV_SINGLE_TAG = VecBuilder.fill(4.0, 4.0, 9999);
    public static final Matrix<N3, N1> STD_DEV_MULTI_TAG = VecBuilder.fill(0.5, 0.5, 999);
    public static final double MAX_TIMESTAMP_DEVIATION = 0.25;
    public static final double MAX_POSITION_DEVIATION = 0.25;
    public static final double MAX_ANGLE_DEVIATION = 5.0;
    public static final int MEDIAN_FILTER_SAMPLES = 7;
    public static final double MAX_SPEED_FOR_VISION_UPDATE_TELEOP = 0.5; //m/s
    public static final double MAX_ROTATION_RATE_FOR_VISION_UPDATE_TELEOP = 30.0; //deg/s
    public static final double NOTE_TIME_OF_FLIGHT = 0.2; // seconds

    public static final double MAX_Z_FOR_POSE3D_ACCEPT = 0.75;

    public static final double AIMPOINT_Y_OFFSET = Units.inchesToMeters(0); // was -12 inches
    public static final Rotation2d AIM_YAW_OFFSET_BLUE = Rotation2d.fromDegrees(5); // CCW+
    public static final Rotation2d AIM_YAW_OFFSET_RED = Rotation2d.fromDegrees(1.5); // CCW+
    public static final double AIMPOINT_X_OFFSET = Units.inchesToMeters(9);

    //public static final double[] DISTANCE_BREAKPOINTS = {3.8, 10, 15, 20};
//    public static final double[] DISTANCE_BREAKPOINTS = {5, 7.5, 10, 15, 17.5, 20, 25};
//    public static final double[] NOTE_FLIGHT_TIMES = {0.25, 0.28, 0.34, 0.4, 0.45, 0.52, 0.63};

    public static final double[] DISTANCE_BREAKPOINTS = {3.8, 10, 15, 20};
    public static final double[] NOTE_FLIGHT_TIMES = {0.18, 0.28, 0.44, 0.46};

    public static double NOTE_FLIGHT_TIME_SCALAR = 1.0;
    // public static final double NOTE_FLIGHT_TIME_SCALAR = .45;

    public static final double[] CROSS_SHOT_DISTANCE_BREAKPOINTS = {22, 25, 30, 33.5};
    public static final double[] CROSS_SHOT_NOTE_FLIGHT_TIMES = {1.5, 1.5, 1.5, 1.5};
    public static double ACCELERATION_SCALAR = 0.0;

    public static final double[] AIM_OFFSET_RED_ANGLES = {0, 0.5*Math.PI, 2.6, 2.8 ,Math.PI};
    // Before tuning at home...
    // public static final double[] AIM_OFFSET_RED_OFFSETS = {1.5, 0, 0, 1.5, 1.5};
    public static final double[] AIM_OFFSET_RED_OFFSETS = {5, 2, 2, 5, 5};

    public static final double[] AIM_OFFSET_BLUE_ANGLES = {-0.5*Math.PI, 0, 0.3, 0.5 ,Math.PI};
    // Pre worlds
    // public static final double[] AIM_OFFSET_BLUE_OFFSETS = {5, 5, 5, 0, 0};
    
    // Worlds, we did this because we removed the angle offset we had in the limelight config
    // This seemed to be a good move, the accuracy of the limelight readings for botpose was improved.
    public static final double[] AIM_OFFSET_BLUE_OFFSETS = {7, 7, 7, 4, 4};

    public static final double NOTE_CAMERA_HEIGHT_M = Units.inchesToMeters(23.5);
    public static final Rotation2d NOTE_CAMERA_ANGLE = Rotation2d.fromDegrees(20);

}
