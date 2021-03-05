// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

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

    private final AHRS navx_device = new AHRS();

    public final WPI_TalonFX talonFXLeft = new WPI_TalonFX(12);
    public final WPI_TalonFX talonFXRight = new WPI_TalonFX(13);
    private final WPI_TalonFX talonFXLeftFollower = new WPI_TalonFX(14);
    private final WPI_TalonFX talonFXRightFollower = new WPI_TalonFX(15);

    public final DifferentialDrive differentialDrive1 = new DifferentialDrive(talonFXLeft, talonFXRight);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(navx_device.getRotation2d());
    private final Field2d m_field = new Field2d();
    private final Faults faultsLeft = new Faults();
    private final Faults faultsRight = new Faults();

    public DriveTrain() {

        initTalonFX(talonFXLeft, TalonFXInvertType.CounterClockwise);
        initTalonFX(talonFXRight, TalonFXInvertType.Clockwise);

        talonFXLeftFollower.configFactoryDefault();
        talonFXLeftFollower.follow(talonFXLeft);
        talonFXLeftFollower.setInverted(InvertType.FollowMaster);

        talonFXRightFollower.configFactoryDefault();
        talonFXRightFollower.follow(talonFXRight);
        talonFXRightFollower.setInverted(InvertType.FollowMaster);
  
        /* Zero the sensor once on robot boot up */
        resetEncoders();

        addChild("Differential Drive 1", differentialDrive1);
        differentialDrive1.setRightSideInverted(false);
        differentialDrive1.setSafetyEnabled(true);
        differentialDrive1.setExpiration(0.1);
        differentialDrive1.setMaxOutput(0.75);
        differentialDrive1.setDeadband(0.02);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(navx_device.getRotation2d(),
                      nativeUnitsToDistanceMeters(talonFXLeft.getSelectedSensorPosition()),
                      nativeUnitsToDistanceMeters(talonFXRight.getSelectedSensorPosition()));
        m_field.setRobotPose(m_odometry.getPoseMeters());

        /* Instrumentation */
        Instrumentation.ProcessNavX(navx_device);
        Instrumentation.ProcessTalon(talonFXRight);
        updateDashboard();
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    
    public void logPeriodic() {

        double leftVelUnitsPer100ms = talonFXLeft.getSelectedSensorVelocity(0);
        double rghtVelUnitsPer100ms = talonFXRight.getSelectedSensorVelocity(0);

        String work = " L:" + leftVelUnitsPer100ms + " R:" + rghtVelUnitsPer100ms;

        talonFXLeft.getFaults(faultsLeft);
        if (faultsLeft.SensorOutOfPhase) {
            work += " L sensor is out of phase";
        }
        talonFXRight.getFaults(faultsRight);
        if (faultsRight.SensorOutOfPhase) {
            work += " R sensor is out of phase";
        }
        System.out.println(work);
    }

    public void updateDashboard()
    {
        SmartDashboard.putData("Field", m_field);
    }

    public void enableBrakes(boolean enabled) {
        talonFXLeft.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
        talonFXRight.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, navx_device.getRotation2d());
    }

    public void resetEncoders() {
        talonFXLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        talonFXRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }

    public double getAverageEncoderDistance() {
        return (nativeUnitsToDistanceMeters(talonFXLeft.getSelectedSensorPosition()) +
                nativeUnitsToDistanceMeters(talonFXRight.getSelectedSensorPosition())) / 2.0;
    }

    public void zeroHeading() {
        navx_device.reset();
    }
    public double getHeadingDegrees() {
        return navx_device.getRotation2d().getDegrees();
    }
    public Rotation2d getHeading() {
        return navx_device.getRotation2d();
    }
    public double getTurnRate() {
        return -navx_device.getRate();
    }

    public void setMaxOutput(double maxOutput) {
        differentialDrive1.setMaxOutput(maxOutput);
    }
    public void setRampRate(double rampTimeSeconds) {
        talonFXLeft.configOpenloopRamp(rampTimeSeconds);
        talonFXRight.configOpenloopRamp(rampTimeSeconds);
    }

    public void initTalonFX(WPI_TalonFX talon, TalonFXInvertType invert)
    {
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
		talonFXLeft.config_kP(Constants.kSlotIdx, kp, Constants.kTimeoutMs);
		talonFXLeft.config_kI(Constants.kSlotIdx, ki, Constants.kTimeoutMs);
		talonFXLeft.config_kD(Constants.kSlotIdx, kd, Constants.kTimeoutMs);
		talonFXLeft.config_kF(Constants.kSlotIdx, kf, Constants.kTimeoutMs);
		talonFXLeft.config_IntegralZone(Constants.kSlotIdx, (int)iZone, Constants.kTimeoutMs);
		talonFXLeft.configMaxIntegralAccumulator(Constants.kSlotIdx, maxIntegral, Constants.kTimeoutMs);

		talonFXRight.config_kP(Constants.kSlotIdx, kp, Constants.kTimeoutMs);
		talonFXRight.config_kI(Constants.kSlotIdx, ki, Constants.kTimeoutMs);
		talonFXRight.config_kD(Constants.kSlotIdx, kd, Constants.kTimeoutMs);
		talonFXRight.config_kF(Constants.kSlotIdx, kf, Constants.kTimeoutMs);
		talonFXRight.config_IntegralZone(Constants.kSlotIdx, (int)iZone, Constants.kTimeoutMs);
		talonFXRight.configMaxIntegralAccumulator(Constants.kSlotIdx, maxIntegral, Constants.kTimeoutMs);
    }
    
    // Put methods for controlling this subsystem here. Call these from Commands.
    public void driveArcade(double speed, double rotation, boolean useSquares) {
        differentialDrive1.arcadeDrive(speed, rotation, useSquares);
    }
    public void driveTank(double leftSpeed, double rightSpeed) {
        differentialDrive1.tankDrive(leftSpeed, rightSpeed);
    }
    public void driveCurvature(double speed, double rotation, boolean quickTurn) {
        differentialDrive1.curvatureDrive(speed, rotation, quickTurn);
    }
    public void driveToTarget(double meters) {
        int targetPos = distanceMetersToNativeUnits(meters);

        talonFXLeft.set(TalonFXControlMode.MotionMagic, targetPos);
        talonFXRight.set(TalonFXControlMode.MotionMagic, targetPos);
    }
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        talonFXLeft.setVoltage(leftVolts);
        talonFXRight.setVoltage(rightVolts);
        differentialDrive1.feed();
    }
    public int distanceMetersToNativeUnits(double positionMeters) {
        double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        double motorRotations = wheelRotations * Constants.kGearRatio;
        int sensorCounts = (int)(motorRotations * kCountsPerRev);
        return sensorCounts;
    }
/*    
    public int velocityToNativeUnits(double velocityMetersPerSecond) {
        double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / Constants.k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
        return sensorCountsPer100ms;
    }
*/    
    public double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = (double)sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / Constants.kGearRatio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        return positionMeters;
    }
}

