// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.DriveMode;

public class CommandDriveTrain extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public final DriveTrain m_driveTrain;
    public final XboxController m_controller;
    public boolean _wasLeftStickDown = false;

    public CommandDriveTrain(DriveTrain driveTrain, XboxController controller) {
        m_driveTrain = driveTrain;
        m_controller = controller;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_driveTrain.setDriveMode(DriveMode.ARCADE);
        m_driveTrain.setUseSquares(true);
        m_driveTrain.enableBrakes(true);
        m_driveTrain.enableDriveTrain(true);
        _wasLeftStickDown = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        boolean leftStickDown = m_controller.getStickButton(Hand.kLeft);
        if (!_wasLeftStickDown && leftStickDown) {
            m_driveTrain.setUseSquares(!m_driveTrain.getUseSquares());
        }
        _wasLeftStickDown = leftStickDown;
        m_driveTrain.setOutput(m_controller);

        //m_driveTrain.logPeriodic();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.tankDriveVolts(0, 0);
        m_driveTrain.enableDriveTrain(false);
        m_driveTrain.setDriveMode(DriveMode.ARCADE);
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
    }
}