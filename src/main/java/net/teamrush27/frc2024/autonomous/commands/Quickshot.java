package net.teamrush27.frc2024.autonomous.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.teamrush27.frc2024.subsystems.launcher.Launcher;
import net.teamrush27.frc2024.subsystems.launcher.LauncherSystemState;
import net.teamrush27.frc2024.subsystems.supervisor.Supervisor;
import net.teamrush27.frc2024.subsystems.supervisor.SupervisorWantedState;


public class Quickshot extends Command {
      // The subsystem the command runs on
      private final Supervisor supervisorInstance;
      private final Launcher launcher;
    
      public Quickshot() {
        supervisorInstance = Supervisor.getInstance();
        launcher = Launcher.getInstance();
      }
    
      @Override
      public void initialize() {
        launcher.setAutonQuickshotTrue();
        supervisorInstance.setWantedState(SupervisorWantedState.FIRE);
      }
    
      @Override
      public boolean isFinished() {
        return launcher.getSystemState().equals(LauncherSystemState.READY_TO_FIRE);
      }
    }