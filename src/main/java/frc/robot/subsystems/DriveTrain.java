// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;


public class DriveTrain extends SubsystemBase {

    private final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.

    private final AHRS navx_device = new AHRS();

    private final WPI_TalonFX talonFXLeft = new WPI_TalonFX(13);
    private final WPI_TalonFX talonFXRight = new WPI_TalonFX(14);
    private final WPI_TalonFX talonFXLeftFollower = new WPI_TalonFX(15);
    private final WPI_TalonFX talonFXRightFollower = new WPI_TalonFX(12);

    private final DifferentialDrive differentialDrive1 = new DifferentialDrive(talonFXLeft, talonFXRight);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(navx_device.getRotation2d());
    private final Field2d m_field = new Field2d();
    private final Faults faultsLeft = new Faults();
    private final Faults faultsRight = new Faults();

    public DriveTrain() {
        talonFXLeft.configFactoryDefault();
        talonFXRight.configFactoryDefault();
        talonFXLeftFollower.configFactoryDefault();
        talonFXRightFollower.configFactoryDefault();

        talonFXLeftFollower.follow(talonFXLeft);
        talonFXRightFollower.follow(talonFXRight);

        //talonFXLeft.setInverted(TalonFXInvertType.CounterClockwise); // !< Update this
        //talonFXRight.setInverted(TalonFXInvertType.Clockwise); // !< Update this
        talonFXLeftFollower.setInverted(InvertType.FollowMaster);
        talonFXRightFollower.setInverted(InvertType.FollowMaster);

        addChild("Differential Drive 1",differentialDrive1);
        differentialDrive1.setSafetyEnabled(true);
        differentialDrive1.setExpiration(0.1);
        differentialDrive1.setMaxOutput(0.75);
        differentialDrive1.setDeadband(0.05);

        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(navx_device.getRotation2d(),
                      nativeUnitsToDistanceMeters(talonFXLeft.getSelectedSensorPosition()),
                      nativeUnitsToDistanceMeters(talonFXRight.getSelectedSensorPosition()));
        m_field.setRobotPose(m_odometry.getPoseMeters());

        SmartDashboard.putBoolean( "IMU_Connected",        navx_device.isConnected());
        SmartDashboard.putBoolean( "IMU_IsCalibrating",    navx_device.isCalibrating());
        SmartDashboard.putNumber(  "IMU_Yaw",              navx_device.getYaw());
        SmartDashboard.putNumber(  "IMU_Pitch",            navx_device.getPitch());
        SmartDashboard.putNumber(  "IMU_Roll",             navx_device.getRoll());
        
        /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
        SmartDashboard.putNumber(  "IMU_CompassHeading",   navx_device.getCompassHeading());

        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        SmartDashboard.putNumber(  "IMU_FusedHeading",     navx_device.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class */
        SmartDashboard.putNumber(  "IMU_TotalYaw",         navx_device.getAngle());
        SmartDashboard.putNumber(  "IMU_YawRateDPS",       navx_device.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        SmartDashboard.putNumber(  "IMU_Accel_X",          navx_device.getWorldLinearAccelX());
        SmartDashboard.putNumber(  "IMU_Accel_Y",          navx_device.getWorldLinearAccelY());
        SmartDashboard.putBoolean( "IMU_IsMoving",         navx_device.isMoving());
        SmartDashboard.putBoolean( "IMU_IsRotating",       navx_device.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  */
        /* not expected to be accurate enough for estimating robot position on a    */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially      */
        /* double (displacement) integration.                                       */        SmartDashboard.putNumber(  "IMU_Temp_C",           navx_device.getTempC());
        SmartDashboard.putNumber(  "Velocity_X",           navx_device.getVelocityX() );
        SmartDashboard.putNumber(  "Velocity_Y",           navx_device.getVelocityY() );
        SmartDashboard.putNumber(  "Displacement_X",       navx_device.getDisplacementX() );
        SmartDashboard.putNumber(  "Displacement_Y",       navx_device.getDisplacementY() );
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */  
        SmartDashboard.putNumber(   "RawGyro_X",           navx_device.getRawGyroX());
        SmartDashboard.putNumber(   "RawGyro_Y",           navx_device.getRawGyroY());
        SmartDashboard.putNumber(   "RawGyro_Z",           navx_device.getRawGyroZ());
        SmartDashboard.putNumber(   "RawAccel_X",          navx_device.getRawAccelX());
        SmartDashboard.putNumber(   "RawAccel_Y",          navx_device.getRawAccelY());
        SmartDashboard.putNumber(   "RawAccel_Z",          navx_device.getRawAccelZ());
        SmartDashboard.putNumber(   "RawMag_X",            navx_device.getRawMagX());
        SmartDashboard.putNumber(   "RawMag_Y",            navx_device.getRawMagY());
        SmartDashboard.putNumber(   "RawMag_Z",            navx_device.getRawMagZ());
        SmartDashboard.putNumber(   "IMU_Temp_C",          navx_device.getTempC());
        
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        AHRS.BoardYawAxis yaw_axis = navx_device.getBoardYawAxis();
        SmartDashboard.putString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(  "YawAxis",              yaw_axis.board_axis.getValue());

        /* Sensor Board Information                                                 */
        SmartDashboard.putString(  "FirmwareVersion",      navx_device.getFirmwareVersion());

        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        SmartDashboard.putNumber(  "QuaternionW",          navx_device.getQuaternionW());
        SmartDashboard.putNumber(  "QuaternionX",          navx_device.getQuaternionX());
        SmartDashboard.putNumber(  "QuaternionY",          navx_device.getQuaternionY());
        SmartDashboard.putNumber(  "QuaternionZ",          navx_device.getQuaternionZ());

        /* Connectivity Debugging Support                                           */
        SmartDashboard.putNumber(  "IMU_Update_Count",     navx_device.getUpdateCount());
        SmartDashboard.putNumber(  "IMU_Byte_Count",       navx_device.getByteCount());
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    
    public void logPeriodic() {

        String work = "";
 
        double leftVelUnitsPer100ms = talonFXLeft.getSelectedSensorVelocity(0);
        double rghtVelUnitsPer100ms = talonFXRight.getSelectedSensorVelocity(0);

        work += " L:" + leftVelUnitsPer100ms + " R:" + rghtVelUnitsPer100ms;
        talonFXLeft.getFaults(faultsLeft);
        talonFXRight.getFaults(faultsRight);
        if (faultsLeft.SensorOutOfPhase) {
            work += " L sensor is out of phase";
        }
        if (faultsRight.SensorOutOfPhase) {
            work += " R sensor is out of phase";
        }
        System.out.println(work);
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
        talonFXLeft.setSelectedSensorPosition(0, 0, 10);
        talonFXRight.setSelectedSensorPosition(0, 0, 10);
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

    public void setClosedLoopGains(double kp, double ki, double kd, double kf, double iZone, double maxIntegral) {
		talonFXLeft.config_kP(0, kp, 0);
		talonFXLeft.config_kI(0, ki, 0);
		talonFXLeft.config_kD(0, kd, 0);
		talonFXLeft.config_kF(0, kf, 0);
		talonFXLeft.config_IntegralZone(0, (int)Constants.kLengthChassisMeters, 30);
		talonFXLeft.configMaxIntegralAccumulator(0, maxIntegral, 0);

		talonFXRight.config_kP(0, kp, 0);
		talonFXRight.config_kI(0, ki, 0);
		talonFXRight.config_kD(0, kd, 0);
		talonFXRight.config_kF(0, kf, 0);
		talonFXRight.config_IntegralZone(0, (int)Constants.kLengthChassisMeters, 30);
		talonFXRight.configMaxIntegralAccumulator(0, maxIntegral, 0);
    }
    
    // Put methods for controlling this subsystem here. Call these from Commands.
    public void driveArcade(double speed, double rotation, boolean useSquares) {
        differentialDrive1.arcadeDrive(speed, rotation, useSquares);
    }
    
    public void driveTank(double leftSpeed, double rightSpeed) {
        differentialDrive1.tankDrive(leftSpeed, rightSpeed);
    }
/*
    private int distanceToNativeUnits(double positionMeters) {
        double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        double motorRotations = wheelRotations * Constants.kGearRatio;
        int sensorCounts = (int)(motorRotations * kCountsPerRev);
        return sensorCounts;
    }
    
    private int velocityToNativeUnits(double velocityMetersPerSecond) {
        double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / Constants.k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
        return sensorCountsPer100ms;
    }
*/    
    private double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = (double)sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / Constants.kGearRatio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        return positionMeters;
    }
}

