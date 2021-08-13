// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class AutoDriveTurnCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

	private final RobotContainer _robot;
	private final DriveTrain _driveTrain;
	private final XboxController _controller;

	/** Config Objects for motor controllers */
	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

	private double _lockedDistance = 0;
	private double _targetAngle = 0;
	private boolean _gotoEnd = true;

	private int _smoothing = 0;

	public AutoDriveTurnCommand(RobotContainer container) {
        _robot = container;
		_driveTrain = _robot.driveTrain;
		_controller = _robot.driverController;

		addRequirements(_driveTrain);
    }
   
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        System.out.println("AutoDriveTurnCommand - initialize");

        _driveTrain.enableDriveTrain(false);
        _driveTrain.enableBrakes(false);

        _driveTrain.leftMaster.getAllConfigs(_leftConfig);
        _driveTrain.rightMaster.getAllConfigs(_rightConfig);

        System.out.println("AutoDriveTurnCommand - leftConfig(before): " + _leftConfig);
        System.out.println("AutoDriveTurnCommand - rightConfig(before): " + _rightConfig);
		
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
		
		_lockedDistance = 5.0;
		_targetAngle = -900.0;  // -360000 * (90.0 / 360.0) * 0.5; //175 degrees
		_gotoEnd = true;

		SmartDashboard.putNumber("Smoothing", _smoothing);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
		System.out.println("AutoDriveTurnCommand - execute");

		if (_controller.getStartButtonPressed()) {
			_gotoEnd = !_gotoEnd;
			_lockedDistance = _gotoEnd ? 5.0 : 0.0;
			_targetAngle = _gotoEnd ? -900.0 : 900.0; //-360000 * (90.0 / 360.0) : 0.0;
		}
		if (_controller.getBumperPressed(Hand.kLeft)) {
			if (--_smoothing < 0) _smoothing = 0; // Cap smoothing
			_driveTrain.rightMaster.configMotionSCurveStrength(_smoothing);
			SmartDashboard.putNumber("Smoothing", _smoothing);
		}
		if (_controller.getBumperPressed(Hand.kRight)) {
			if (++_smoothing > 8) _smoothing = 8; // Cap smoothing
			_driveTrain.rightMaster.configMotionSCurveStrength(_smoothing);
			SmartDashboard.putNumber("Smoothing", _smoothing);
		}

		/* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
		_driveTrain.rightMaster.setTarget(_lockedDistance, _targetAngle);
		_driveTrain.leftMaster.follow(_driveTrain.rightMaster, FollowerType.AuxOutput1);
		_driveTrain.logPeriodic();
		
		SmartDashboard.putNumber("PoseX", _driveTrain.getCurrentPose().getTranslation().getX());
		SmartDashboard.putNumber("PoseY", _driveTrain.getCurrentPose().getTranslation().getY());
		SmartDashboard.putNumber("Pose Rot", _driveTrain.getCurrentPose().getRotation().getDegrees());
		System.out.println("target (meters) = " + _lockedDistance + " angle: " + _targetAngle);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("AutoDriveTurnCommand - end");
        _driveTrain.enableDriveTrain(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        double error = _driveTrain.rightMaster.getClosedLoopError();
        System.out.println("AutoDriveTurnCommand - error: " + error);

        return false;
    }

 
}    