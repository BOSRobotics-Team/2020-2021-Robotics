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
    public DriveMode m_Mode = DriveMode.ARCADE;

    private final DriveTrain m_driveTrain;
    private final XboxController m_controller;

    public CommandDriveTrain(DriveTrain driveTrain, XboxController controller) {
        m_driveTrain = driveTrain;
        m_controller = controller;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_Mode = DriveMode.ARCADE;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (m_controller.getBackButtonPressed()) {
            switch (m_Mode) {
                case ARCADE: 
                    m_Mode = DriveMode.TANK;
                    break;
                case TANK:
                    m_Mode = DriveMode.ARCADE;
                    break;
                default:
                    m_Mode = DriveMode.ARCADE;
                    break;
            }
        }
        if (m_Mode == DriveMode.ARCADE) {
            m_driveTrain.driveArcade(-m_controller.getY(Hand.kLeft), m_controller.getX(Hand.kRight), true);
        } else if (m_Mode == DriveMode.TANK) {
            m_driveTrain.driveTank(-m_controller.getY(Hand.kLeft), m_controller.getY(Hand.kRight));
        }

        m_driveTrain.logPeriodic();

        SmartDashboard.putString(  "DriveTrainMode",   m_Mode.toString());
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.driveArcade(0, 0, true);
        m_Mode = DriveMode.ARCADE;
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
   }
}
