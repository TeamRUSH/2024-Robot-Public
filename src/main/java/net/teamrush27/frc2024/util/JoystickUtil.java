package net.teamrush27.frc2024.util;

public class JoystickUtil {
  public static double deadband(double value, double zero) {
    if (Math.abs(value) < zero) {
      return 0;
    }

    return value;
  }
}
