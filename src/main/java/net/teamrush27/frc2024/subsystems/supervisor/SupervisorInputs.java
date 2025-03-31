package net.teamrush27.frc2024.subsystems.supervisor;

import monologue.Annotations.*;
import monologue.Logged;
import net.teamrush27.frc2024.subsystems.indexer.IndexerSystemState;
import net.teamrush27.frc2024.subsystems.intake.IntakeSystemState;
import net.teamrush27.frc2024.subsystems.launcher.LauncherSystemState;


public class SupervisorInputs implements Logged {
    public IntakeSystemState intakeState;
    public IndexerSystemState indexerState;
    public LauncherSystemState launcherState;
    @Log
    public double timeInState;
}
