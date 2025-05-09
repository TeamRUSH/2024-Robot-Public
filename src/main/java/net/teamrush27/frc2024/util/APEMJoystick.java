package net.teamrush27.frc2024.util;

import edu.wpi.first.wpilibj.Joystick;

public class APEMJoystick extends Joystick {

  public APEMJoystick(int port) {
    super(port);
  }

  public double getXAxis() {return super.getX() * -1;}
  public double getYAxis() {return super.getY() * -1;}
  public double getZAxis() {return super.getZ() * -1;}

  public boolean getLeftButton() {
    return super.getRawButton(2);
  }

  public boolean getRightButton() {
    return super.getRawButton(1);
  }

  public boolean getRightButtonPressed() {
    return super.getRawButtonPressed(1);
  }

  public boolean getLeftButtonPressed() {
    return super.getRawButtonPressed(2);
  }

  public static double deadband(double value, double zero) {
    if (Math.abs(value) < zero) {
      return 0;
    }

    return value;
  }
}
