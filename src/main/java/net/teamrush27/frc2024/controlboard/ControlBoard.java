package net.teamrush27.frc2024.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Annotations.*;

public class ControlBoard implements IDriverControlBoard, IOperatorControlBoard {
    @IgnoreLogged
    private static ControlBoard instance;

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final IDriverControlBoard driverControlBoard;
    private final IOperatorControlBoard operatorControlBoard;

    private ControlBoard() {
        driverControlBoard = DriverControlBoard.getInstance();
        operatorControlBoard = OperatorControlBoard.getInstance();
    }

    public IDriverControlBoard getDriverControlBoard() {
        return driverControlBoard;
    }

    @Override
    public double getThrottle() {
        return driverControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return driverControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return driverControlBoard.getRotation();
    }

    @Override
    public Trigger getResetHeading() {
        return driverControlBoard.getResetHeading();
    }

    @Override
    public Trigger getRobotOriented() {
        return driverControlBoard.getRobotOriented();
    }

    @Override
    public Trigger getSpeakerTracking() {
        return driverControlBoard.getSpeakerTracking();
    }

    @Override
    public Trigger getIntake() {
        return operatorControlBoard.getIntake();
    }

    @Override
    public Trigger getExhaust() {
        return operatorControlBoard.getExhaust();
    }

    @Override
    public Trigger getResetIntakePosition() {
        return operatorControlBoard.getResetIntakePosition();
    }

    @Override
    public Trigger getLeaveIntakeOut() {
        return operatorControlBoard.getLeaveIntakeOut();
    }

    @Override
    public Trigger getShotOverride() {
        return operatorControlBoard.getShotOverride();
    }

    @Override
    public Trigger getFire() {
        return operatorControlBoard.getFire();
    }

    @Override
    public Trigger getSpinUp() {
        return operatorControlBoard.getSpinUp();
    }

    @Override
    public Trigger getSubwooferShot() {
        return operatorControlBoard.getSubwooferShot();
    }

    @Override
    public Trigger getPodiumShot() {
        return operatorControlBoard.getPodiumShot();
    }

    @Override
    public Trigger getAmpShot() {
        return operatorControlBoard.getAmpShot();
    }

    @Override
    public Trigger getNorth() {
        return operatorControlBoard.getNorth();
    }

    @Override
    public Trigger getSouth() {
        return operatorControlBoard.getSouth();
    }

    @Override
    public Trigger getSourceShot1() {
        return driverControlBoard.getSourceShot1();
    }
    @Override
    public Trigger getSourceShot2() {
        return driverControlBoard.getSourceShot2();
    }
    @Override
    public Trigger getCrossFieldShot() {
        return driverControlBoard.getCrossFieldShot();
    }
    @Override
    public Trigger getNoteTracking() {
        return driverControlBoard.getNoteTracking();
    }

    @Override
    public Trigger incrementCrankAngle() {
        return driverControlBoard.incrementCrankAngle();
    }

    @Override
    public Trigger decrementCrankAngle() {
       return driverControlBoard.decrementCrankAngle();
    }
}
