package net.teamrush27.frc2024.controlboard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlBoard implements IDriverControlBoard {
    public static DriverControlBoard instance = null;

    public static DriverControlBoard getInstance() {
        if (instance == null) {
            instance = new DriverControlBoard();
        }
        return instance;
    }

    private final XboxController controller;

    private DriverControlBoard() {
        controller = new XboxController(Constants.DRIVER_GAMEPAD_PORT);
    }

    // Curved and deadbanded; up positive and down negative
    @Override
    public double getThrottle() {
        return -controller.getLeftY();
    }

    // Curved and deadbanded, Left positive and right negative
    @Override
    public double getStrafe() {
        return -controller.getLeftX();
    }

    // Curved and deadbanded; Left positive and right negative
    @Override
    public double getRotation() {
        return -controller.getRightX();
    }

    @Override
    public Trigger getResetHeading() {
        return new Trigger(controller::getXButton);
    }

    @Override
    public Trigger getRobotOriented() {
        return new Trigger(controller::getAButton);
    }

    @Override
    public Trigger getSpeakerTracking() {
        return new Trigger(() -> controller.getRightTriggerAxis() > 0.12);
    }

    @Override
    public Trigger getSourceShot1() {
        return new Trigger(controller::getLeftBumper);
    }
    @Override
    public Trigger getSourceShot2() {
        return new Trigger(() -> controller.getLeftTriggerAxis() > 0.12);
    }

    @Override
    public Trigger getCrossFieldShot() { return new Trigger(controller::getRightBumper);}
    @Override
    public Trigger getNoteTracking() {
        return new Trigger(controller::getBButton);
    }

    @Override
    public Trigger incrementCrankAngle(){
        return new Trigger(controller::getYButton);
    }

    @Override
    public Trigger decrementCrankAngle(){
        return new Trigger(controller::getAButton);
    }
}
