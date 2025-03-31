package net.teamrush27.frc2024.util;

import edu.wpi.first.wpilibj.RobotController;
import net.teamrush27.frc2024.Constants;

public class RobotHelper {

    public enum RobotType {
        COMPETITION,
        PRACTICE,
        SIMULATION;
    }

    public static RobotType getRobotType() {
        String serialNumber = RobotController.getSerialNumber();
        return switch (serialNumber) {
            case Constants.PRACTICE_SERIAL -> RobotType.PRACTICE;
            case Constants.COMPETITION_SERIAL -> RobotType.COMPETITION;
            default -> RobotType.SIMULATION;
        };
    }

}
