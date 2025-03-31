package net.teamrush27.frc2024.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IOperatorControlBoard {
    Trigger getIntake();

    Trigger getExhaust();
    Trigger getResetIntakePosition();
    Trigger getLeaveIntakeOut();
    Trigger getShotOverride();

    Trigger getFire();

    Trigger getSpinUp();

    Trigger getSubwooferShot();

    Trigger getPodiumShot();

    Trigger getAmpShot();

    Trigger getNorth();

    Trigger getSouth();
}
