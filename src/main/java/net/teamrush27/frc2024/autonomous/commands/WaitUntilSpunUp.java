package net.teamrush27.frc2024.autonomous.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.teamrush27.frc2024.subsystems.launcher.Launcher;
import net.teamrush27.frc2024.subsystems.launcher.LauncherWantedState;

public class WaitUntilSpunUp extends Command {

    Launcher launcher = Launcher.getInstance();
    @Override
    public void initialize() {
        launcher.setWantedState(LauncherWantedState.SPINUP);
    }

    @Override
    public boolean isFinished() {
        return launcher.isReady();
    }
}
