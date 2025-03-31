// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package net.teamrush27.frc2024;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import monologue.Logged;
import monologue.Annotations.Log;

public class RobotContainer implements Logged{

    @Log
    private SendableChooser<Command> autonChooser;

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public void configureAuton() {
        autonChooser = new SendableChooser<>();
        autonChooser.addOption("Do Nothing", Commands.print("Doing Nothing."));

//        autonChooser.addOption("2-6-3... Kettering", new PathPlannerAuto("0-2-6 Kettering"));

        autonChooser.addOption("Center 4 Note", new PathPlannerAuto("4 Note"));
        autonChooser.addOption("Center 5 Rush", new PathPlannerAuto("Hail Mary"));
        autonChooser.addOption("Center 6 Rush BEE", new PathPlannerAuto("Bee Hail Mary"));
        autonChooser.addOption("Center 6 Lame BEE", new PathPlannerAuto("WBGC Center"));

        autonChooser.addOption("AMP Normal", new PathPlannerAuto("Fixed AMP Normal"));
        autonChooser.addOption("AMP Swapped", new PathPlannerAuto("Fixed AMP Swapped"));

        autonChooser.addOption("Source 0-8-7", new PathPlannerAuto("SOURCE 0-8-7 Around"));
        autonChooser.addOption("Source 0-7-8", new PathPlannerAuto("SOURCE 0-7-8 Around"));
        autonChooser.addOption("Source 0-7-6", new PathPlannerAuto("SOURCE 0-7-6 Stage"));
        autonChooser.addOption("Source 8-7-0", new PathPlannerAuto("SOURCE 8-7-0 Around"));
        autonChooser.addOption("Source 8-6-0", new PathPlannerAuto("SOURCE 8-6-0 Stage"));

        autonChooser.setDefaultOption("Do Nothing", Commands.print("You messed up skibidi \n-Steve"));

        SmartDashboard.putData(autonChooser);
    }
}
