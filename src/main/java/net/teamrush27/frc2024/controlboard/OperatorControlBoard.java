package net.teamrush27.frc2024.controlboard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorControlBoard implements IOperatorControlBoard{
    private static OperatorControlBoard instance;
    public static OperatorControlBoard getInstance() {
        if(instance == null) {
            instance = new OperatorControlBoard();
        }
        return instance;
    }

    private final XboxController controller;

    private OperatorControlBoard() {
        controller =  new XboxController(Constants.OPERATOR_GAMEPAD_PORT);
    }

    @Override
    public Trigger getIntake() {
        return new Trigger(() -> controller.getLeftTriggerAxis() > 0.12);
    }

    @Override
    public Trigger getExhaust() {
        return new Trigger(controller::getLeftBumper);
    }

    @Override
    public Trigger getResetIntakePosition() {
        return new Trigger(controller::getStartButton);
    }

    @Override
    public Trigger getLeaveIntakeOut() {
        return new Trigger(controller::getBackButton);
    }

    @Override
    public Trigger getShotOverride() {
        return new Trigger(controller::getStartButton);
    }

    @Override
    public Trigger getFire() {
        return new Trigger(() -> controller.getRightTriggerAxis() > 0.12);
    }

    @Override
    public Trigger getSpinUp() {
        return new Trigger(controller::getRightBumper);
    }

    @Override
    public Trigger getSubwooferShot() {
        return new Trigger(controller::getYButton);
    }

    @Override
    public Trigger getPodiumShot() {
        return new Trigger(controller::getXButton);
    }

    @Override
    public Trigger getAmpShot() {
        return new Trigger(controller::getAButton);
    }

    @Override
    public Trigger getNorth() {
        return new Trigger(() -> controller.getPOV() == 0);
    }

    @Override
    public Trigger getSouth() {
        return new Trigger(() -> controller.getPOV() == 180);
    }

}
