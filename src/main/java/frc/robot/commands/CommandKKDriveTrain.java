// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;

public class CommandKKDriveTrain extends CommandDriveTrain {
    
    public CommandKKDriveTrain(DriveTrain driveTrain, XboxController controller) {
        super(driveTrain, controller);

    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        m_driveTrain.drive(m_controller);

        //m_driveTrain.logPeriodic();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_driveTrain.setDriveScaling(1.0);
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return super.isFinished();
    }   
}