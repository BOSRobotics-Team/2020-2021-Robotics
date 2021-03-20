// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.DriveMode;

public class CommandKKDriveTrain extends CommandDriveTrain {
    
    public enum SmoothingOption {
        OPTION1,
        OPTION2,
        OPTION3,
        OPTION4
    }
    private SmoothingOption _selectedSmoothingOption = SmoothingOption.OPTION1;
    private double _lastX1, _lastX2 = 0.0;
    private double _lastY1, _lastY2 = 0.0;
    private boolean wasBButtonPressed = false;


    public CommandKKDriveTrain(DriveTrain driveTrain, XboxController controller) {
        super(driveTrain, controller);

    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        super.initialize();

        _lastX1 = _lastX2 = _lastY1 = _lastY2 = 0.0;
        wasBButtonPressed = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        m_driveTrain.drive(m_controller);

        boolean bButton = m_controller.getBButtonPressed();

        if (bButton && !wasBButtonPressed)
            toggleSmoothing();

        double yLeft = -m_controller.getY(Hand.kLeft);
        double xRight = (m_driveTrain.getDriveMode() == DriveMode.TANK) ? -m_controller.getY(Hand.kRight) : m_controller.getX(Hand.kRight);

        double x = 0.0;
        double y = 0.0;

        switch (_selectedSmoothingOption) {
            case OPTION1:
                x = (xRight + _lastX1) / 2.0;
                y = (yLeft + _lastY1) / 2.0;
                break;
            case OPTION2:
                x = (xRight + _lastX1 + _lastX2) / 3.0;
                y = (yLeft + _lastY1 + _lastY2) / 3.0;
                break;
            case OPTION3:
                x = (Math.pow(xRight, 2) * Math.signum(xRight));
                y = (Math.pow(yLeft, 2) * Math.signum(yLeft));
                break;
            case OPTION4:
                x = Math.sqrt(((Math.pow(xRight, 2) + Math.pow(_lastX1, 2)) * 0.5)) * Math.signum(xRight + _lastX1);
                y = Math.sqrt(((Math.pow(yLeft, 2) + Math.pow(_lastY1, 2)) * 0.5)) * Math.signum(yLeft + _lastY1);
                break;
        }
        m_driveTrain.setOutput(x,y);

        _lastX2 = _lastX1;
        _lastX1 = xRight;
        _lastY2 = _lastY1;
        _lastY1 = yLeft;

        //m_driveTrain.logPeriodic();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return super.isFinished();
    }   

    public void toggleSmoothing() {
        switch (_selectedSmoothingOption)
        {
            case OPTION1: _selectedSmoothingOption = SmoothingOption.OPTION2; break;
            case OPTION2: _selectedSmoothingOption = SmoothingOption.OPTION3; break;
            case OPTION3: _selectedSmoothingOption = SmoothingOption.OPTION4; break;
            case OPTION4: _selectedSmoothingOption = SmoothingOption.OPTION1; break;
        }
    }
}