/**
 * Instrumentation Class that handles how telemetry from the Talon FX interacts
 * with Driverstation and Smart Dashboard.
 */
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

public class Instrumentation {
	/* Tracking variables for instrumentation */
	private static int _timesInMotionMagic = 0;
	private static int _timesInProcessTalon = 0;
	private static int _timesInProcessNavX = 0;

	public static void ProcessTalon(TalonFX tal) {
        if (++_timesInProcessTalon > 9)
        {
            /* Smart dash plots */
            SmartDashboard.putNumber("SensorVel", tal.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
            SmartDashboard.putNumber("SensorPos", tal.getSelectedSensorPosition(Constants.kPIDLoopIdx));
            SmartDashboard.putNumber("MotorOutputPercent", tal.getMotorOutputPercent());
            SmartDashboard.putNumber("ClosedLoopError", tal.getClosedLoopError(Constants.kPIDLoopIdx));
            
            /* Check if Talon FX is performing Motion Magic */
            if (tal.getControlMode() == ControlMode.MotionMagic) {
                ++_timesInMotionMagic;
            } else {
                _timesInMotionMagic = 0;
            }

            if (_timesInMotionMagic > 10) {
                /* Print the Active Trajectory Point Motion Magic is servoing towards */
                SmartDashboard.putNumber("ClosedLoopTarget", tal.getClosedLoopTarget(Constants.kPIDLoopIdx));
                SmartDashboard.putNumber("ActTrajVelocity", tal.getActiveTrajectoryVelocity());
                SmartDashboard.putNumber("ActTrajPosition", tal.getActiveTrajectoryPosition());
            }
            _timesInProcessTalon = 0;
        }
    }
    public static void ProcessNavX(AHRS navx) {
        if (++_timesInProcessNavX > 9)
        {
            /* Smart dash plots */
            SmartDashboard.putBoolean( "IMU_Connected",        navx.isConnected());
            SmartDashboard.putBoolean( "IMU_IsCalibrating",    navx.isCalibrating());
            SmartDashboard.putNumber(  "IMU_Yaw",              navx.getYaw());
            SmartDashboard.putNumber(  "IMU_Pitch",            navx.getPitch());
            SmartDashboard.putNumber(  "IMU_Roll",             navx.getRoll());
            
            /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
            SmartDashboard.putNumber(  "IMU_CompassHeading",   navx.getCompassHeading());

            /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
            SmartDashboard.putNumber(  "IMU_FusedHeading",     navx.getFusedHeading());

            /* These functions are compatible w/the WPI Gyro Class */
            SmartDashboard.putNumber(  "IMU_TotalYaw",         navx.getAngle());
            SmartDashboard.putNumber(  "IMU_YawRateDPS",       navx.getRate());

            /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
            SmartDashboard.putNumber(  "IMU_Accel_X",          navx.getWorldLinearAccelX());
            SmartDashboard.putNumber(  "IMU_Accel_Y",          navx.getWorldLinearAccelY());
            SmartDashboard.putBoolean( "IMU_IsMoving",         navx.isMoving());
            SmartDashboard.putBoolean( "IMU_IsRotating",       navx.isRotating());

            /* Display estimates of velocity/displacement.  Note that these values are  */
            /* not expected to be accurate enough for estimating robot position on a    */
            /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
            /* of these errors due to single (velocity) integration and especially      */
            /* double (displacement) integration.                                       */        
            SmartDashboard.putNumber(  "IMU_Temp_C",           navx.getTempC());
            SmartDashboard.putNumber(  "Velocity_X",           navx.getVelocityX() );
            SmartDashboard.putNumber(  "Velocity_Y",           navx.getVelocityY() );
            SmartDashboard.putNumber(  "Displacement_X",       navx.getDisplacementX() );
            SmartDashboard.putNumber(  "Displacement_Y",       navx.getDisplacementY() );
            
            /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
            /* NOTE:  These values are not normally necessary, but are made available   */
            /* for advanced users.  Before using this data, please consider whether     */
            /* the processed data (see above) will suit your needs.                     */  
            SmartDashboard.putNumber(   "RawGyro_X",           navx.getRawGyroX());
            SmartDashboard.putNumber(   "RawGyro_Y",           navx.getRawGyroY());
            SmartDashboard.putNumber(   "RawGyro_Z",           navx.getRawGyroZ());
            SmartDashboard.putNumber(   "RawAccel_X",          navx.getRawAccelX());
            SmartDashboard.putNumber(   "RawAccel_Y",          navx.getRawAccelY());
            SmartDashboard.putNumber(   "RawAccel_Z",          navx.getRawAccelZ());
            SmartDashboard.putNumber(   "RawMag_X",            navx.getRawMagX());
            SmartDashboard.putNumber(   "RawMag_Y",            navx.getRawMagY());
            SmartDashboard.putNumber(   "RawMag_Z",            navx.getRawMagZ());
            SmartDashboard.putNumber(   "IMU_Temp_C",          navx.getTempC());
            
            /* Omnimount Yaw Axis Information                                           */
            /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
            AHRS.BoardYawAxis yaw_axis = navx.getBoardYawAxis();
            SmartDashboard.putString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
            SmartDashboard.putNumber(  "YawAxis",              yaw_axis.board_axis.getValue());

            /* Sensor Board Information                                                 */
            SmartDashboard.putString(  "FirmwareVersion",      navx.getFirmwareVersion());

            /* Quaternion Data                                                          */
            /* Quaternions are fascinating, and are the most compact representation of  */
            /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
            /* from the Quaternions.  If interested in motion processing, knowledge of  */
            /* Quaternions is highly recommended.                                       */
            SmartDashboard.putNumber(  "QuaternionW",          navx.getQuaternionW());
            SmartDashboard.putNumber(  "QuaternionX",          navx.getQuaternionX());
            SmartDashboard.putNumber(  "QuaternionY",          navx.getQuaternionY());
            SmartDashboard.putNumber(  "QuaternionZ",          navx.getQuaternionZ());

            /* Connectivity Debugging Support                                           */
            SmartDashboard.putNumber(  "IMU_Update_Count",     navx.getUpdateCount());
            SmartDashboard.putNumber(  "IMU_Byte_Count",       navx.getByteCount());   
        }
        _timesInProcessNavX = 0;
    }
}