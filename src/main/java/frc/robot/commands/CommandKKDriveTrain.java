// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.DriveMode;

public class CommandKKDriveTrain extends CommandDriveTrain {

    private boolean _lastTriggerL = false;
    private boolean _lastTriggerR = false;

    private boolean scalingOn = false;
    private double scaling = 0.5;

    private double _lastX = 0.0;
    private double _lastY = 0.0;
    
    public CommandKKDriveTrain(DriveTrain driveTrain, XboxController controller) {
        super(driveTrain, controller);

    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();

        _lastX = _lastY = 0.0;
        _lastTriggerL = _lastTriggerR = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double triggerL = m_controller.getTriggerAxis(Hand.kLeft);
        double triggerR = m_controller.getTriggerAxis(Hand.kRight);
        boolean leftStickDown = m_controller.getStickButton(Hand.kLeft);
        boolean rightStickDown = m_controller.getStickButton(Hand.kRight);

        if ((triggerL > 0.0) && !_lastTriggerL)
        { 
            scaling = Math.min(scaling + 0.1, 1.0);
            m_driveTrain.setDriveScaling(scalingOn ? scaling : 1.0);
        }
        _lastTriggerL = (triggerL > 0.0);

        if ((triggerR > 0.0) && !_lastTriggerR)
        {
            scaling = Math.max(scaling - 0.1, 0.1);
            m_driveTrain.setDriveScaling(scalingOn ? scaling : 1.0);
        }
        _lastTriggerR = (triggerR > 0.0);

        if (!_wasLeftStickDown && leftStickDown) {
            scalingOn = !scalingOn;
            m_driveTrain.setDriveScaling(scalingOn ? scaling : 1.0);
        }
        _wasLeftStickDown = leftStickDown;

        double yLeft = -m_controller.getY(Hand.kLeft) * m_driveTrain.getDriveScaling();
        double xRight = _lastX;

        if (m_driveTrain.getDriveMode() == DriveMode.TANK)
            xRight = -m_controller.getY(Hand.kRight) * m_driveTrain.getDriveScaling();
        else
            xRight = m_controller.getX(Hand.kRight) * m_driveTrain.getDriveScaling();

        double x = (xRight + _lastX) / 2.0;
        double y = (yLeft + _lastY) / 2.0;

        if (m_driveTrain.getDriveMode() == DriveMode.ARCADE) {
            m_driveTrain.driveArcade(y, x, m_driveTrain.getUseSquares());
        } else if (m_driveTrain.getDriveMode() == DriveMode.TANK) {
            m_driveTrain.driveTank(y, x);
        } else if (m_driveTrain.getDriveMode() == DriveMode.CURVATURE) {
            m_driveTrain.driveCurvature(y, x, rightStickDown);
        }

//        m_driveTrain.setOutput(m_controller);

        //m_driveTrain.logPeriodic();
        _lastX = xRight;
        _lastY = yLeft;
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