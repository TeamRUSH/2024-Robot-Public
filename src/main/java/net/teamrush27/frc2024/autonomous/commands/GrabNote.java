package net.teamrush27.frc2024.autonomous.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.teamrush27.frc2024.subsystems.supervisor.Supervisor;
import net.teamrush27.frc2024.subsystems.supervisor.SupervisorSystemState;
import net.teamrush27.frc2024.subsystems.supervisor.SupervisorWantedState;


public class GrabNote extends Command {
      // The subsystem the command runs on
      private final Supervisor supervisorInstance;

      boolean isGrabNoteFinished = false;

      public GrabNote() {
        supervisorInstance = Supervisor.getInstance();
        // addRequirements(supervisorInstance);
      }
    
      @Override
      public void initialize() {
        supervisorInstance.setWantedState(SupervisorWantedState.INTAKE);
      }
    
      @Override
      public boolean isFinished() {
        isGrabNoteFinished = supervisorInstance.getCurrentState().equals(SupervisorSystemState.LOADED);
        return isGrabNoteFinished;
      }
    }