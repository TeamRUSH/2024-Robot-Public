package net.teamrush27.frc2024.autonomous.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.teamrush27.frc2024.subsystems.drivetrain.Drivetrain;

public class AimAtSpeaker extends Command {

    Drivetrain drivetrain = Drivetrain.getInstance();
    @Override
    public void initialize() {
        drivetrain.setWantedState(Drivetrain.WantedState.SPEAKER_ALIGN);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.isAimed(3);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setWantedState(Drivetrain.WantedState.AUTON);
    }
}
