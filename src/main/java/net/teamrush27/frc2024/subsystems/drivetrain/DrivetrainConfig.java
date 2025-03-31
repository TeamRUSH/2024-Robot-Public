package net.teamrush27.frc2024.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import net.teamrush27.frc2024.util.OneDimensionalLookup;

public class DrivetrainConfig {

    public static final double[] JOYSTICK_LATERAL_AXIS_INPUTS =  {-1, -0.85, -0.6, -0.12, 0.12, 0.6, 0.85, 1};
    public static final double[] JOYSTICK_LATERAL_AXIS_OUTPUTS = {-1.0, -0.6, -0.3, 0, 0, 0.3, 0.6, 1.0};
    public static final double[] JOYSTICK_ROTATIONAL_AXIS_INPUTS =  {-1, -0.9, -0.6, -0.12, 0.12, 0.6, 0.9, 1};
    public static final double[] JOYSTICK_ROTATIONAL_AXIS_OUTPUTS = {-1.0, -0.5, -0.2, 0, 0, 0.2, 0.5, 1.0};

    public static final double JOYSTICK_DEADBAND = 0.12;

    public static final double AUTO_AIM_ANGLE_TOLERANCE_DEG = 3;

    // radians
    public static final double SPEAKER_AIM_YAW_TOLERANCE = Units.degreesToRadians(10);

    public static double lookupLateralJoystickOutput(double inputValue) {
        return OneDimensionalLookup.interpLinear(
                JOYSTICK_LATERAL_AXIS_INPUTS,
                JOYSTICK_LATERAL_AXIS_OUTPUTS,
                inputValue
        );
    }

    public static double lookupRotationalJoystickOutput(double inputValue) {
        return OneDimensionalLookup.interpLinear(
                JOYSTICK_ROTATIONAL_AXIS_INPUTS,
                JOYSTICK_ROTATIONAL_AXIS_OUTPUTS,
                inputValue
        );
    }

    public static double applyJoystickFunction(double inputValue, double pow) {
        if(Math.abs(inputValue) <= JOYSTICK_DEADBAND) return 0;

        double result = Math.abs(Math.pow(inputValue, pow));
        result -= Math.pow(JOYSTICK_DEADBAND, pow);
        result /= 1 - Math.pow(JOYSTICK_DEADBAND, pow);
        result *= Math.signum(inputValue);
        return result;
    }


    public static final ProfiledPIDController autoAimController = new ProfiledPIDController(
            8,
            5,
            1,
            new TrapezoidProfile.Constraints(4*Math.PI,16*Math.PI));

    public static final double AUTO_AIM_CONTROLLER_IZONE = 0.5;

}
