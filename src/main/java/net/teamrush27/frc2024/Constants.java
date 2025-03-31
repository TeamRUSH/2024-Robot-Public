package net.teamrush27.frc2024;


import edu.wpi.first.math.util.Units;

public class Constants {
        public static String CANBUS_NAME = "CANFD";
        public static double RECHECK_DEVICE_INTERVAL_SECONDS = 10;


        // Drivetrain Calibrations
        public static final double DRIVETRAIN_RADIUS = Math.sqrt(2) * Units.inchesToMeters(11.625);
        public static final double MAX_LINEAR_VELOCITY_MPS = Units.feetToMeters(17.3);
        public static final double MAX_ROTATIONAL_VELOCITY_RPS = MAX_LINEAR_VELOCITY_MPS / DRIVETRAIN_RADIUS;
        public static final double CAPPED_ROTATIONAL_VELOCITY_RPS = 4 * Math.PI;

        public static double kSwerveHeadingControllerErrorTolerance = 1;
        public static double kSnapRadiusKp = 1;
        public static double kSnapRadiusKi = .1;
        public static double kSnapRadiusKd = .5;
        public static double kMaintainRadiusKp = .3;
        public static double kMaintainRadiusKi = .5;
        public static double kMaintainRadiusKd = .1;

        public static final String COMPETITION_SERIAL = "03224427";
        public static final String PRACTICE_SERIAL = "032B1FA3";

}
