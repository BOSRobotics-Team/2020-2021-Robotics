package frc.robot.wrappers;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

public class DriveGyro extends AHRS {
    
    /** Whether or not to use the NavX for driving straight */
    private boolean overrideGyro = false;

    /**
     * Set the robot's heading.
     * @param heading The heading to set to, in degrees on [-180, 180].
     */
    public void setHeadingDegrees(final double heading) {
        this.setAngleAdjustment(heading);
    }

    public double getHeadingDegrees() {
        return this.getRotation2d().getDegrees();
    }
    
    public Rotation2d getHeading() {
        return this.getRotation2d();
    }

    /**
     * Zero the robot's heading.
     */
    public void zeroHeading() {
        this.reset();
    }

    /**
     * Get the robot's angular velocity.
     * @return Angular velocity in degrees/sec
     */
    public double getAngularVel() {
        return -this.getRate();
    }

    /**
     * Get the robot's angular displacement since being turned on.
     * @return Angular displacement in degrees.
     */
    public double getAngularDisplacement() {
        return -this.getAngle();
    }

    /** @return true if the NavX is currently overriden, false otherwise. */
    public boolean getOverrideGyro() {
        return overrideGyro;
    }

    /** @param override true to override the NavX, false to un-override it. */
    public void setOverrideGyro(final boolean override) {
        overrideGyro = override;
    }

    public void logPeriodic() {
        /* Smart dash plots */
        SmartDashboard.putBoolean( "IMU_Connected",        this.isConnected());
        SmartDashboard.putBoolean( "IMU_IsCalibrating",    this.isCalibrating());
        SmartDashboard.putNumber(  "IMU_Yaw",              this.getYaw());
        SmartDashboard.putNumber(  "IMU_Pitch",            this.getPitch());
        SmartDashboard.putNumber(  "IMU_Roll",             this.getRoll());
        
        /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
        SmartDashboard.putNumber(  "IMU_CompassHeading",   this.getCompassHeading());

        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        SmartDashboard.putNumber(  "IMU_FusedHeading",     this.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class */
        SmartDashboard.putNumber(  "IMU_TotalYaw",         this.getAngle());
        SmartDashboard.putNumber(  "IMU_YawRateDPS",       this.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        SmartDashboard.putNumber(  "IMU_Accel_X",          this.getWorldLinearAccelX());
        SmartDashboard.putNumber(  "IMU_Accel_Y",          this.getWorldLinearAccelY());
        SmartDashboard.putBoolean( "IMU_IsMoving",         this.isMoving());
        SmartDashboard.putBoolean( "IMU_IsRotating",       this.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  */
        /* not expected to be accurate enough for estimating robot position on a    */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially      */
        /* double (displacement) integration.                                       */        
        SmartDashboard.putNumber(  "IMU_Temp_C",           this.getTempC());
        SmartDashboard.putNumber(  "Velocity_X",           this.getVelocityX() );
        SmartDashboard.putNumber(  "Velocity_Y",           this.getVelocityY() );
        SmartDashboard.putNumber(  "Displacement_X",       this.getDisplacementX() );
        SmartDashboard.putNumber(  "Displacement_Y",       this.getDisplacementY() );
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */  
        SmartDashboard.putNumber(   "RawGyro_X",           this.getRawGyroX());
        SmartDashboard.putNumber(   "RawGyro_Y",           this.getRawGyroY());
        SmartDashboard.putNumber(   "RawGyro_Z",           this.getRawGyroZ());
        SmartDashboard.putNumber(   "RawAccel_X",          this.getRawAccelX());
        SmartDashboard.putNumber(   "RawAccel_Y",          this.getRawAccelY());
        SmartDashboard.putNumber(   "RawAccel_Z",          this.getRawAccelZ());
        SmartDashboard.putNumber(   "RawMag_X",            this.getRawMagX());
        SmartDashboard.putNumber(   "RawMag_Y",            this.getRawMagY());
        SmartDashboard.putNumber(   "RawMag_Z",            this.getRawMagZ());
        SmartDashboard.putNumber(   "IMU_Temp_C",          this.getTempC());
        
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        AHRS.BoardYawAxis yaw_axis = this.getBoardYawAxis();
        SmartDashboard.putString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(  "YawAxis",              yaw_axis.board_axis.getValue());

        /* Sensor Board Information                                                 */
        SmartDashboard.putString(  "FirmwareVersion",      this.getFirmwareVersion());

        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        SmartDashboard.putNumber(  "QuaternionW",          this.getQuaternionW());
        SmartDashboard.putNumber(  "QuaternionX",          this.getQuaternionX());
        SmartDashboard.putNumber(  "QuaternionY",          this.getQuaternionY());
        SmartDashboard.putNumber(  "QuaternionZ",          this.getQuaternionZ());

        /* Connectivity Debugging Support                                           */
        SmartDashboard.putNumber(  "IMU_Update_Count",     this.getUpdateCount());
        SmartDashboard.putNumber(  "IMU_Byte_Count",       this.getByteCount());   
    }
}
