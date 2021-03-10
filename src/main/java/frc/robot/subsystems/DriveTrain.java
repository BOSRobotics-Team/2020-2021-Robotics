// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import javax.annotation.Nullable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.Instrumentation;

public class DriveTrain extends SubsystemBase {

    private final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.

    public final WPI_TalonFX leftMaster = new WPI_TalonFX(12);
    public final WPI_TalonFX rightMaster = new WPI_TalonFX(13);
    private final WPI_TalonFX leftFollower = new WPI_TalonFX(14);
    private final WPI_TalonFX rightFollower = new WPI_TalonFX(15);

    /** The NavX gyro */
    private final AHRS ahrs = new AHRS();

    /** Drivetrain kinematics processor for measuring individual wheel speeds */
    private final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Constants.kWidthChassisMeters);

    /** Drivetrain odometry tracker for tracking position */
    private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(ahrs.getRotation2d());

    /** Whether or not to use the NavX for driving straight */
    private boolean overrideGyro = false;

    public final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

    public SimpleMotorFeedforward leftFeedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);
    public SimpleMotorFeedforward rightFeedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);

    private final Field2d m_field = new Field2d();
    private final Faults faultsLeft = new Faults();
    private final Faults faultsRight = new Faults();

    /** The most recently set setpoint. */
    private double leftSetpoint;
    private double rightSetpoint;

    /** The setpoint in native units. Field to avoid garbage collection. */
    private double leftNativeSetpoint;
    private double rightNativeSetpoint;

//    private boolean voltageCompEnabled = false;
    private Double maxSpeed;

    /** Cached values for various sensor readings. */
    private double cachedLeftVel = Double.NaN;
    private double cachedRightVel = Double.NaN;
    private double cachedLeftPos = Double.NaN;
    private double cachedRightPos = Double.NaN;


    public DriveTrain() {
        initTalonFX(leftMaster, TalonFXInvertType.CounterClockwise);
        initTalonFX(rightMaster, TalonFXInvertType.Clockwise);

        leftFollower.configFactoryDefault();
        leftFollower.follow(leftMaster);
        leftFollower.setInverted(InvertType.FollowMaster);

        rightFollower.configFactoryDefault();
        rightFollower.follow(rightMaster);
        rightFollower.setInverted(InvertType.FollowMaster);
  
        /* Zero the sensor once on robot boot up */
        resetPosition();

        addChild("Differential Drive 1", differentialDrive);
        differentialDrive.setRightSideInverted(false);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(0.75);
        differentialDrive.setDeadband(0.02);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateOdometry();

        /* Instrumentation */
        Instrumentation.ProcessNavX(ahrs);
        Instrumentation.ProcessTalon(rightMaster);
        updateDashboard();
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    
    /**
     * Set the output of each side of the drive.
     *
     * @param left The output for the left side of the drive, from [-1, 1]
     * @param right the output for the right side of the drive, from [-1, 1]
     */
    void setOutput(double leftVelocity, double rightVelocity) {
        // scale by the max speed
        if (maxSpeed != null) {
            setVelocityUPS(leftVelocity * maxSpeed, rightVelocity * maxSpeed);
        } else {
            setPercentVoltage(leftVelocity, rightVelocity);
        }
    }

    /**
     * Get the velocity of the left side of the drive.
     *
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    Double getLeftVel() {
        return nativeUnitsToVelocity(leftMaster.getSelectedSensorVelocity(0));
    }

    /**
     * Get the velocity of the right side of the drive.
     *
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    Double getRightVel() {
        return nativeUnitsToVelocity(rightMaster.getSelectedSensorVelocity(0));
    }

    /**
     * Get the position of the left side of the drive.
     *
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    @Nullable
    Double getLeftPos() {
        return nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition(0));
    }

    /**
     * Get the position of the right side of the drive.
     *
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    @Nullable
    Double getRightPos() {
        return nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition(0));
    }

    /**
     * Get the cached velocity of the left side of the drive.
     *
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    Double getLeftVelCached() {
        return cachedLeftVel;
    }

    /**
     * Get the cached velocity of the right side of the drive.
     *
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    Double getRightVelCached() {
        return cachedRightVel;
    }

    /**
     * Get the cached position of the left side of the drive.
     *
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    @Nullable
    Double getLeftPosCached() {
        return cachedLeftPos;
    }

    /**
     * Get the cached position of the right side of the drive.
     *
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    @Nullable
    Double getRightPosCached() {
        return cachedRightPos;
    }

    /** @return The feedforward calculator for left motors */
    public SimpleMotorFeedforward getLeftFeedforwardCalculator() {
        return leftFeedForwardCalculator;
    }

    /** @return The feedforward calculator for right motors */
    public SimpleMotorFeedforward getRightFeedforwardCalculator() {
        return rightFeedForwardCalculator;
    }
    
    /** Completely stop the robot by setting the voltage to each side to be 0. */
    public void fullStop() {
        setPercentVoltage(0, 0);
    }
  
    /**
     * Set the robot's heading.
     *
     * @param heading The heading to set to, in degrees on [-180, 180].
     */
    public void setHeadingDegrees(final double heading) {
        ahrs.setAngleAdjustment(heading);
    }

    /**
     * Get the robot's angular velocity.
     *
     * @return Angular velocity in degrees/sec
     */
    public double getAngularVel() {
        return -ahrs.getRate();
    }

    /**
     * Get the robot's angular displacement since being turned on.
     *
     * @return Angular displacement in degrees.
     */
    public double getAngularDisplacement() {
        return -ahrs.getAngle();
    }

    /**
     * Get the pitch value.
     *
     * @return The pitch, in degrees from [-180, 180]
     */
    public double getPitch() {
        return ahrs.getPitch();
    }

    /** @return true if the NavX is currently overriden, false otherwise. */
    public boolean getOverrideGyro() {
        return overrideGyro;
    }

    /** @param override true to override the NavX, false to un-override it. */
    public void setOverrideGyro(final boolean override) {
        overrideGyro = override;
    }

    /** Reset odometry tracker to current robot pose */
    public void resetOdometry(final Pose2d pose) {
        resetPosition();
        setHeadingDegrees(pose.getRotation().getDegrees());
        driveOdometry.resetPosition(pose, getHeading());
//        driveOdometry.resetPosition(pose, ahrs.getRotation2d());
    }

    /** Update odometry tracker with current heading, and encoder readings */
    public void updateOdometry() {
        // need to convert to meters
        driveOdometry.update(getHeading(), 
                             getLeftPos(), 
                             getRightPos());
    
//        driveOdometry.update(ahrs.getRotation2d(),
//                      nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition()),
//                      nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition()));
 //       m_field.setRobotPose(driveOdometry.getPoseMeters());
    }

    /** @return Current estimated pose based on odometry tracker data */
    public Pose2d getCurrentPose() {
        return driveOdometry.getPoseMeters() != null
            ? driveOdometry.getPoseMeters()
            : new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    }

      /** @return Current wheel speeds based on encoder readings for future pose correction */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // need to convert to meters
        return new DifferentialDriveWheelSpeeds(getLeftVel(), getRightVel());
    }

    /** @return Kinematics processor for wheel speeds */
    public DifferentialDriveKinematics getDriveKinematics() {
        return driveKinematics;
    }

    /** Disable the motors. */
    public void disable() {
        leftMaster.disable();
        rightMaster.disable();
    }

    /**
     * Hold the current position.
     *
     * @param pos the position to stop at
     */
    public void holdPosition(final double pos) {
        holdPosition(pos, pos);
    }

    /** Updates all cached values with current ones. */
    public void update() {
        cachedLeftVel = getLeftVel();
        cachedLeftPos = getLeftPos();
        cachedRightVel = getRightVel();
        cachedRightPos = getRightPos();
    }

    /**
     * Hold the current position.
     *
     * @param leftPos the position to stop the left side at
     * @param rightPos the position to stop the right side at
     */
    public void holdPosition(final double leftPos, final double rightPos) {
        leftSetpoint = leftPos;
        rightSetpoint = rightPos;
    }

    public void setLeftPercentVoltage(double pctVolts) {
    // Warn the user if they're setting Vbus to a number that's outside the range of values.
        if (Math.abs(pctVolts) > 1.0) {
            Shuffleboard.addEventMarker(
                "WARNING: YOU ARE CLIPPING MAX PERCENT VBUS AT L:" + pctVolts,
                getClass().getSimpleName(),
                EventImportance.kNormal);

                pctVolts = Math.signum(pctVolts);
        }
        leftSetpoint = pctVolts;
        leftMaster.set(ControlMode.PercentOutput, pctVolts);
    }
    public void setRightPercentVoltage(double pctVolts) {
        // Warn the user if they're setting Vbus to a number that's outside the range of values.
        if (Math.abs(pctVolts) > 1.0) {
            Shuffleboard.addEventMarker(
                "WARNING: YOU ARE CLIPPING MAX PERCENT VBUS AT R:" + pctVolts,
                getClass().getSimpleName(),
                EventImportance.kNormal);
                pctVolts = Math.signum(pctVolts);
        }
        rightSetpoint = pctVolts;
        rightMaster.set(ControlMode.PercentOutput, pctVolts);
    }
    public void setPercentVoltage(double leftPctVolts, double rightPctVolts) {
        setLeftPercentVoltage(leftPctVolts);
        setRightPercentVoltage(rightPctVolts);
    } 

    public void setVelocity(final double leftVel, final double rightVel) {
        if (maxSpeed != null) {
            setVelocityUPS(leftVel * maxSpeed, rightVel * maxSpeed);
        } else {
            setPercentVoltage(leftVel, rightVel);   
        }
    }
    public void setVelocityUPS(final double leftVel, final double rightVel) {
        leftNativeSetpoint = velocityToNativeUnits(leftVel);
        rightNativeSetpoint = velocityToNativeUnits(rightVel);
        leftSetpoint = leftVel;
        rightSetpoint = rightVel;

        leftMaster.config_kF(0, 0, 0);
        leftMaster.set(
            ControlMode.Velocity,
            leftNativeSetpoint,
            DemandType.ArbitraryFeedForward,
            leftFeedForwardCalculator.calculate(leftVel) / 12.);
        rightMaster.config_kF(0, 0, 0);
        rightMaster.set(
            ControlMode.Velocity,
            rightNativeSetpoint,
            DemandType.ArbitraryFeedForward,
            rightFeedForwardCalculator.calculate(rightVel) / 12.);
    }

    public double getLeftSetpoint() {
        return leftSetpoint;
    }
    public double getRightSetpoint() {
        return rightSetpoint;
    }

    public double getLeftOutputVoltage() {
        return leftMaster.getMotorOutputVoltage();
    }
    public double getRightOutputVoltage() {
        return rightMaster.getMotorOutputVoltage();
    }
    public double getLeftBatteryVoltage() {
        return leftMaster.getBusVoltage();
    }
    public double getRightBatteryVoltage() {
        return rightMaster.getBusVoltage();
    }
    public double getLeftOutputCurrent() {
        return leftMaster.getSupplyCurrent();
    }
    public double getRightOutputCurrent() {
        return rightMaster.getSupplyCurrent();
    }
    public String getLeftControlMode() {
        return leftMaster.getControlMode().name();
    }
    public String getRightControlMode() {
        return rightMaster.getControlMode().name();
    }
      /** Resets the position of the Talon to 0. */
    public void resetPosition() {
        leftMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rightMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }
           
    public void logPeriodic() {
        double leftVelUnitsPer100ms = leftMaster.getSelectedSensorVelocity(0);
        double rghtVelUnitsPer100ms = rightMaster.getSelectedSensorVelocity(0);

        String work = " L:" + leftVelUnitsPer100ms + " R:" + rghtVelUnitsPer100ms;

        leftMaster.getFaults(faultsLeft);
        if (faultsLeft.SensorOutOfPhase) {
            work += " L sensor is out of phase";
        }
        rightMaster.getFaults(faultsRight);
        if (faultsRight.SensorOutOfPhase) {
            work += " R sensor is out of phase";
        }
        System.out.println(work);
    }

    public void updateDashboard()
    {
        SmartDashboard.putData("Field", m_field);
    }

    public void enableDriveTrain(boolean enable) {
        differentialDrive.setSafetyEnabled(enable);
        leftMaster.set(enable ? ControlMode.PercentOutput : ControlMode.Disabled, 0);
        rightMaster.set(enable ? ControlMode.PercentOutput : ControlMode.Disabled, 0);
    }

    public void enableBrakes(boolean enabled) {
        leftMaster.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
        rightMaster.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public double getAverageEncoderDistance() {
        return (nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition()) +
                nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition())) / 2.0;
    }

    public void zeroHeading() {
        ahrs.reset();
    }
    public double getHeadingDegrees() {
        return ahrs.getRotation2d().getDegrees();
    }
    public Rotation2d getHeading() {
        return ahrs.getRotation2d();
    }

    public void setMaxOutput(double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
    }
    public void setRampRate(double rampTimeSeconds) {
        leftMaster.configOpenloopRamp(rampTimeSeconds);
        rightMaster.configOpenloopRamp(rampTimeSeconds);
    }

    public void initTalonFX(WPI_TalonFX talon, TalonFXInvertType invert) {
        talon.configFactoryDefault();

		/* Configure Sensor Source for Primary PID */
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
        talon.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

 		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
         */
        //talon.setSensorPhase(false);
        talon.setInverted(invert);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, Constants.kTimeoutMs);
        
        /* Set the peak and nominal outputs */
		talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		talon.configPeakOutputForward(1, Constants.kTimeoutMs);
        talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        
		/* Set Motion Magic gains in slot0 - see documentation */
		talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		talon.config_kP(Constants.kSlotIdx, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
		talon.config_kI(Constants.kSlotIdx, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
		talon.config_kD(Constants.kSlotIdx, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
		talon.config_kF(Constants.kSlotIdx, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
		talon.config_IntegralZone(Constants.kSlotIdx, (int)Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
		talon.configMaxIntegralAccumulator(Constants.kSlotIdx, 1.0, Constants.kTimeoutMs);

   		/* Set acceleration and vcruise velocity - see documentation */
        talon.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
        talon.configMotionAcceleration(6000, Constants.kTimeoutMs);
    }

    public void setClosedLoopGains(double kp, double ki, double kd, double kf, double iZone, double maxIntegral) {
		leftMaster.config_kP(Constants.kSlotIdx, kp, Constants.kTimeoutMs);
		leftMaster.config_kI(Constants.kSlotIdx, ki, Constants.kTimeoutMs);
		leftMaster.config_kD(Constants.kSlotIdx, kd, Constants.kTimeoutMs);
		leftMaster.config_kF(Constants.kSlotIdx, kf, Constants.kTimeoutMs);
		leftMaster.config_IntegralZone(Constants.kSlotIdx, (int)iZone, Constants.kTimeoutMs);
		leftMaster.configMaxIntegralAccumulator(Constants.kSlotIdx, maxIntegral, Constants.kTimeoutMs);

		rightMaster.config_kP(Constants.kSlotIdx, kp, Constants.kTimeoutMs);
		rightMaster.config_kI(Constants.kSlotIdx, ki, Constants.kTimeoutMs);
		rightMaster.config_kD(Constants.kSlotIdx, kd, Constants.kTimeoutMs);
		rightMaster.config_kF(Constants.kSlotIdx, kf, Constants.kTimeoutMs);
		rightMaster.config_IntegralZone(Constants.kSlotIdx, (int)iZone, Constants.kTimeoutMs);
		rightMaster.configMaxIntegralAccumulator(Constants.kSlotIdx, maxIntegral, Constants.kTimeoutMs);
    }
    
    // Put methods for controlling this subsystem here. Call these from Commands.
    public void driveArcade(double speed, double rotation, boolean useSquares) {
        differentialDrive.arcadeDrive(speed, rotation, useSquares);
    }
    public void driveTank(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }
    public void driveCurvature(double speed, double rotation, boolean quickTurn) {
        differentialDrive.curvatureDrive(speed, rotation, quickTurn);
    }
    public void driveToTarget(double meters) {
        int targetPos = distanceMetersToNativeUnits(meters);

        leftMaster.set(TalonFXControlMode.MotionMagic, targetPos);
        rightMaster.set(TalonFXControlMode.MotionMagic, targetPos);
    }
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
        differentialDrive.feed();
    }
    public int distanceMetersToNativeUnits(double positionMeters) {
        double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        double motorRotations = wheelRotations * Constants.kGearRatio;
        int sensorCounts = (int)(motorRotations * kCountsPerRev);
        return sensorCounts;
    }
    public double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = (double)sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / Constants.kGearRatio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        return positionMeters;
    }

    public int velocityToNativeUnits(double velocityMetersPerSecond) {
        double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / Constants.k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
        return sensorCountsPer100ms;
    }    
    public double nativeUnitsToVelocity(double sensorCountsPer100ms) {
        double motorRotationsPer100ms = (double)sensorCountsPer100ms / kCountsPerRev;
        double motorRotationsPerSecond = motorRotationsPer100ms * Constants.k100msPerSecond;
        double wheelRotationsPerSecond = motorRotationsPerSecond / Constants.kGearRatio;
        double velocityMetersPerSecond = wheelRotationsPerSecond * (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        return velocityMetersPerSecond;
    }    
}

