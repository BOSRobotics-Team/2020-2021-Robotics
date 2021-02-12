// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class SpinUpShooterCommand extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_shooter;

    public SpinUpShooterCommand(Shooter shooter) {
        m_shooter = shooter;

        addRequirements(m_shooter);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
       
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
       m_shooter.setmotorspeed(0.64);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
       
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}