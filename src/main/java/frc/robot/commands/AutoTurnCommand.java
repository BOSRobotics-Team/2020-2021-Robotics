// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import oi.limelightvision.limelight.frc.LimeLight;
import frc.robot.subsystems.*;

public class AutoTurnCommand extends CommandBase {
@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final DriveTrain m_driveTrain;
    private final LimeLight m_limelight;

    public AutoTurnCommand(DriveTrain drivetrain, LimeLight limelight) {
        m_driveTrain = drivetrain;
        m_limelight = limelight;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double degOff = m_limelight.getdegRotationToTarget();
        if (degOff < 1)
        {
        
        }
    }

     // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {}

   // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
   }

}