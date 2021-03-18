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

    private boolean _wasLeftStickDown = false;
    private boolean _scalingOn = false;
    private double _scaling = 0.5;

    private boolean _lastTriggerL = false;
    private boolean _lastTriggerR = false;

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
        m_driveTrain.setDriveScaling(_scalingOn ? _scaling : 1.0);
        m_driveTrain.enableBrakes(true);
        m_driveTrain.enableDriveTrain(true);

        _wasLeftStickDown = false;
        _lastTriggerL = _lastTriggerR = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        boolean leftStickDown = m_controller.getStickButton(Hand.kLeft);
        if (leftStickDown && !_wasLeftStickDown) {
            _scalingOn = !_scalingOn;
            m_driveTrain.setDriveScaling(_scalingOn ? _scaling : 1.0);
        }
        _wasLeftStickDown = leftStickDown;

        double triggerL = m_controller.getTriggerAxis(Hand.kLeft);
        if ((triggerL >= 0.5) && !_lastTriggerL) { 
            _scaling = Math.min(_scaling + 0.1, 1.0);
            m_driveTrain.setDriveScaling(_scalingOn ? _scaling : 1.0);
        }
        _lastTriggerL = (triggerL > 0.5);

        double triggerR = m_controller.getTriggerAxis(Hand.kRight);
        if ((triggerR >= 0.5) && !_lastTriggerR)
        {
            _scaling = Math.max(_scaling - 0.1, 0.1);
            m_driveTrain.setDriveScaling(_scalingOn ? _scaling : 1.0);
        }
        _lastTriggerR = (triggerR > 0.5);

        m_driveTrain.drive(m_controller);

        //m_driveTrain.logPeriodic();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.tankDriveVolts(0, 0);
        m_driveTrain.setUseSquares(true);
        m_driveTrain.enableBrakes(true);
        m_driveTrain.setDriveScaling(1.0);
        m_driveTrain.setDriveMode(DriveMode.ARCADE);
        m_driveTrain.enableDriveTrain(false);
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
    }
}