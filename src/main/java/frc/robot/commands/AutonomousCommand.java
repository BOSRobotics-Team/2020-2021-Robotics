// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class AutonomousCommand extends CommandBase {
@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final RobotContainer _robot;
    private final DriveTrain _driveTrain;

    /** Invert Directions for Left and Right */
    TalonFXInvertType _leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
    TalonFXInvertType _rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"
    /** Config Objects for motor controllers */
	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

    double _lockedDistance = 0;
	double _targetAngle = 0;
	int _smoothing;

    public AutonomousCommand(RobotContainer container) {
        _robot = container;
        _driveTrain = _robot.driveTrain;

		addRequirements(_driveTrain);
    }
   
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        System.out.println("AutonomousCommand - initialize");

        _driveTrain.enableDriveTrain(false);
        _driveTrain.enableBrakes(false);

        _driveTrain.leftMaster.getAllConfigs(_leftConfig);
        _driveTrain.rightMaster.getAllConfigs(_rightConfig);

        System.out.println("AutonomousCommand - leftConfig(before): " + _leftConfig);
        System.out.println("AutonomousCommand - rightConfig(before): " + _rightConfig);

		_driveTrain.leftMaster.setDistanceConfigs(_leftConfig, Constants.kGains_Distanc);
		_driveTrain.rightMaster.setDistanceConfigs(_rightConfig, Constants.kGains_Distanc);
		_driveTrain.rightMaster.setTurnConfigs(_rightConfig, Constants.kGains_Turning);

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		_rightConfig.remoteFilter0.remoteSensorDeviceID = _driveTrain.leftMaster.getDeviceID(); // Device ID of Source
		_rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; // Remote Feedback Source
		
        System.out.println("AutoDriveTurnCommand - LeftConfig(to set): " + _leftConfig);
        System.out.println("AutoDriveTurnCommand - RightConfig(to set): " + _rightConfig);
		_driveTrain.leftMaster.configAllSettings(_leftConfig);
        _driveTrain.rightMaster.configAllSettings(_rightConfig);
        
		_driveTrain.leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

		_driveTrain.rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, Constants.kTimeoutMs);
		_driveTrain.rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_driveTrain.rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10, Constants.kTimeoutMs);
		_driveTrain.rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, Constants.kTimeoutMs);
		_driveTrain.rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);

		/* Determine which slot affects which PID */
        _driveTrain.rightMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
        _driveTrain.rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
		
		_driveTrain.leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_driveTrain.rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_driveTrain.setHeadingDegrees(0);
		
		SmartDashboard.putNumber("Smoothing", _smoothing);

        _targetAngle = _driveTrain.rightMaster.getSelectedSensorPosition(1);
        _lockedDistance = _driveTrain.rightMaster.getSelectedSensorPosition(0);
   
        System.out.println("_lockedDistance = " + _lockedDistance + " _targetAngle = " + _targetAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        System.out.println("AutonomousCommand - execute");

		/* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
		_driveTrain.rightMaster.setTarget(_lockedDistance, _targetAngle);
		_driveTrain.leftMaster.follow(_driveTrain.rightMaster, FollowerType.AuxOutput1);
        _driveTrain.differentialDrive.feed(); 
		_driveTrain.logPeriodic();
		
		SmartDashboard.putNumber("PoseX", _driveTrain.getCurrentPose().getTranslation().getX());
		SmartDashboard.putNumber("PoseY", _driveTrain.getCurrentPose().getTranslation().getY());
		SmartDashboard.putNumber("Pose Rot", _driveTrain.getCurrentPose().getRotation().getDegrees());
		System.out.println("target (meters) = " + _lockedDistance + " angle: " + _targetAngle);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("AutonomousCommand - end");
        _driveTrain.enableDriveTrain(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
		double error = _driveTrain.rightMaster.getClosedLoopError();
		double velocity = _driveTrain.rightMaster.getActiveTrajectoryVelocity();
        System.out.println("AutonomousCommand - error: " + error + " vel: " + velocity);

        return false;
    }
}    