// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

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

        System.out.println("AutonomousCommand - constructor");
        //addParallel(container.m_spinupShooterCommand);
        //addSequential(container.m_runHopperAutoCommand);
    }
   
    // Called just before this Command runs the first time
    
    @Override
    public void initialize() {
        System.out.println("AutonomousCommand - initialize");

        _driveTrain.driveTank(0, 0);
        _driveTrain.enableBrakes(true);


        		/* Configure the left Talon's selected sensor as local Integrated Sensor */
		_leftConfig.primaryPID.selectedFeedbackSensor =	TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();	// Local Feedback Source

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		_rightConfig.remoteFilter0.remoteSensorDeviceID = _driveTrain.talonFXLeft.getDeviceID(); // Device ID of Source
		_rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; // Remote Feedback Source
		
		/* Now that the Left sensor can be used by the master Talon,
		 * set up the Left (Aux) and Right (Master) distance into a single
		 * Robot distance as the Master's Selected Sensor 0. */
		setRobotDistanceConfigs(_rightInvert, _rightConfig);
		
		/* Setup difference signal to be used for turn when performing Drive Straight with encoders */
		setRobotTurnConfigs(_rightInvert, _rightConfig);

		/* Configure neutral deadband */
		_rightConfig.neutralDeadband = Constants.kNeutralDeadband;
		_leftConfig.neutralDeadband = Constants.kNeutralDeadband;
		
		/* Motion Magic Configurations */
		_rightConfig.motionAcceleration = 2000;
		_rightConfig.motionCruiseVelocity = 2000;

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftConfig.peakOutputForward = +1.0;
		_leftConfig.peakOutputReverse = -1.0;
		_rightConfig.peakOutputForward = +1.0;
		_rightConfig.peakOutputReverse = -1.0;

		/* FPID Gains for distance servo */
		_rightConfig.slot0.kP = Constants.kGains_Distanc.kP;
		_rightConfig.slot0.kI = Constants.kGains_Distanc.kI;
		_rightConfig.slot0.kD = Constants.kGains_Distanc.kD;
		_rightConfig.slot0.kF = Constants.kGains_Distanc.kF;
		_rightConfig.slot0.integralZone = Constants.kGains_Distanc.kIzone;
		_rightConfig.slot0.closedLoopPeakOutput = Constants.kGains_Distanc.kPeakOutput;
		_rightConfig.slot0.allowableClosedloopError = 0;

		/* FPID Gains for turn servo */
		_rightConfig.slot1.kP = Constants.kGains_Turning.kP;
		_rightConfig.slot1.kI = Constants.kGains_Turning.kI;
		_rightConfig.slot1.kD = Constants.kGains_Turning.kD;
		_rightConfig.slot1.kF = Constants.kGains_Turning.kF;
		_rightConfig.slot1.integralZone = Constants.kGains_Turning.kIzone;
		_rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_Turning.kPeakOutput;
		_rightConfig.slot1.allowableClosedloopError = 0;

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

		_driveTrain.talonFXLeft.configAllSettings(_leftConfig);
        _driveTrain.talonFXRight.configAllSettings(_rightConfig);
        
        /* Configure output and sensor direction */
		_driveTrain.talonFXLeft.setInverted(_leftInvert);
        _driveTrain.talonFXRight.setInverted(_rightInvert);

		/* Set status frame periods to ensure we don't have stale data */
		_driveTrain.talonFXRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_driveTrain.talonFXRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_driveTrain.talonFXRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
		_driveTrain.talonFXRight.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
		_driveTrain.talonFXLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

		/* Initialize */
		_driveTrain.talonFXRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
		_driveTrain.resetEncoders();
		
        /* Determine which slot affects which PID */
        _driveTrain.talonFXRight.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
        _driveTrain.talonFXRight.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);

        _targetAngle = _driveTrain.talonFXRight.getSelectedSensorPosition(1);
        _lockedDistance = _driveTrain.talonFXRight.getSelectedSensorPosition(0);

        /* Calculate targets from gamepad inputs */
        double target_sensorUnits = Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel + _lockedDistance;
        double target_turn = _targetAngle;

        System.out.println("target_sensorUnits = " + target_sensorUnits + " target_turn = " + target_turn);

        /* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
        _driveTrain.talonFXRight.set(TalonFXControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn);
        _driveTrain.talonFXLeft.follow(_driveTrain.talonFXRight, FollowerType.AuxOutput1);        
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        System.out.println("AutonomousCommand - execute");
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("AutonomousCommand - end");
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig) {
		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise) {
			masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Diff0 - Diff1
		} else {
			/* Master is not inverted, both sides are positive so we can sum them. */
			masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
		}

		/* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double  the real-world value */
		masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
	 }
	
	void setRobotTurnConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig) {
		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise) {
			masterConfig.sum0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
			masterConfig.auxPIDPolarity = true;
		} else {
			/* Master is not inverted, both sides are positive so we can diff them. */
			masterConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
			/* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
			masterConfig.auxPIDPolarity = true;
		}
		masterConfig.auxiliaryPID.selectedFeedbackCoefficient = Constants.kEncoderUnitsPerRotation / Constants.kEncoderUnitsPerRotation;
	 }
}    