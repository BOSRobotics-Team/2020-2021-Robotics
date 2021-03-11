// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class CommandDriveTrain extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public enum DriveMode {
        ARCADE,
        TANK,
        CURVATURE
    }
    public DriveMode m_DriveMode = DriveMode.ARCADE;
    public boolean m_UseSquares = true;

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
        setDriveMode(DriveMode.ARCADE);
        setUseSquares(true);
        m_driveTrain.enableBrakes(true);
        m_driveTrain.enableDriveTrain(true);
        _wasLeftStickDown = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        boolean leftStickDown = m_controller.getStickButton(Hand.kLeft);
        if (!_wasLeftStickDown && leftStickDown) {
            setUseSquares(!m_UseSquares);
        }
        _wasLeftStickDown = leftStickDown;

        if (m_DriveMode == DriveMode.ARCADE) {
            m_driveTrain.driveArcade(-m_controller.getY(Hand.kLeft), m_controller.getX(Hand.kRight), m_UseSquares);
        } else if (m_DriveMode == DriveMode.TANK) {
            m_driveTrain.driveTank(-m_controller.getY(Hand.kLeft), -m_controller.getY(Hand.kRight));
        } else if (m_DriveMode == DriveMode.CURVATURE) {
            m_driveTrain.driveCurvature(-m_controller.getY(Hand.kLeft), m_controller.getX(Hand.kRight), m_controller.getStickButton(Hand.kRight));
        }

        //m_driveTrain.logPeriodic();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.tankDriveVolts(0, 0);
        m_driveTrain.enableDriveTrain(false);
        setDriveMode(DriveMode.ARCADE);
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
    }
    
    public DriveMode getDriveMode() {
        return m_DriveMode;
    }

    public void setDriveMode(DriveMode mode) {
        m_DriveMode = mode;
        SmartDashboard.putString("DriveTrainMode",   m_DriveMode.toString());
    }
    public void setUseSquares(boolean use) {
        m_UseSquares = use;
        SmartDashboard.putBoolean("UseSquares", m_UseSquares);
    }
 
    public void toggleDriveMode() {
        switch (getDriveMode()) {
            case ARCADE:    setDriveMode(DriveMode.TANK);       break;
            case TANK:      setDriveMode(DriveMode.CURVATURE);  break;
            case CURVATURE: setDriveMode(DriveMode.ARCADE);     break;
            default:    break;
        }
    }
}