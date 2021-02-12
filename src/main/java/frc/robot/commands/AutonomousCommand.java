// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutonomousCommand extends CommandBase {
@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public AutonomousCommand(RobotContainer container) {
        
        //addParallel(container.m_spinupShooterCommand);
        //addSequential(container.m_runHopperAutoCommand);
    }
}    