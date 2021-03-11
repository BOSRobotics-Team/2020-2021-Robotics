// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class CommandKKDriveTrain extends CommandDriveTrain {

    public double scaling = 1.0;

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
        boolean leftStickDown = m_controller.getStickButton(Hand.kLeft);
        if (!_wasLeftStickDown && leftStickDown) {
             if (scaling == 1.0) 
                scaling = 0.5;
            else
                scaling = 1.0;
        }
        _wasLeftStickDown = leftStickDown;

        double y = -m_controller.getY(Hand.kLeft) * scaling;
        double y2 = -m_controller.getY(Hand.kRight) * scaling;
        double x = m_controller.getY(Hand.kRight) * scaling;

        if (m_DriveMode == DriveMode.ARCADE) {
            m_driveTrain.driveArcade(y, x, m_UseSquares);
        } else if (m_DriveMode == DriveMode.TANK) {
            m_driveTrain.driveTank(y, y2);
        } else if (m_DriveMode == DriveMode.CURVATURE) {
            m_driveTrain.driveCurvature(y, x, m_controller.getStickButton(Hand.kRight));
        }

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
}