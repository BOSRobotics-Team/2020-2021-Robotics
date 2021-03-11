// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.annotation.Nullable;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.Instrumentation;
import frc.robot.wrappers.SmartMotor;

public class DriveTrain extends SubsystemBase {

    public final SmartMotor leftMaster = new SmartMotor(12, TalonFXInvertType.CounterClockwise);
    public final SmartMotor rightMaster = new SmartMotor(13, TalonFXInvertType.Clockwise);
    private final WPI_TalonFX leftFollower = new WPI_TalonFX(14);
    private final WPI_TalonFX rightFollower = new WPI_TalonFX(15);

    /** The NavX gyro */
    private final AHRS ahrs = new AHRS();

    /** Drivetrain kinematics processor for measuring individual wheel speeds */
    private final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Constants.kWidthChassisMeters);

    /** Drivetrain odometry tracker for tracking position */
    private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(ahrs.getRotation2d());
    public final DifferentialDrive differentialDrive;

    /** Whether or not to use the NavX for driving straight */
    private boolean overrideGyro = false;

    private final Field2d m_field = new Field2d();

//    private boolean voltageCompEnabled = false;
    private Double maxSpeed;

    public DriveTrain() {

        leftFollower.configFactoryDefault();
        leftFollower.follow(leftMaster.getTalon());
        leftFollower.setInverted(InvertType.FollowMaster);

        rightFollower.configFactoryDefault();
        rightFollower.follow(rightMaster.getTalon());
        rightFollower.setInverted(InvertType.FollowMaster);
  
        /* Zero the sensor once on robot boot up */
        resetPosition();

        differentialDrive = new DifferentialDrive(leftMaster.getTalon(), rightMaster.getTalon());
        differentialDrive.setRightSideInverted(false);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(0.75);
        differentialDrive.setDeadband(0.02);

        addChild("Differential Drive 1", differentialDrive);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateOdometry();

        /* Instrumentation */
        Instrumentation.ProcessNavX(ahrs);
        Instrumentation.ProcessTalon(rightMaster.getTalon());
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
    public void setVelocity(double leftVelocity, double rightVelocity) {
        // scale by the max speed
        if (maxSpeed != null) {
            leftMaster.setVelocityUPS(leftVelocity * maxSpeed);
            rightMaster.setVelocityUPS(rightVelocity * maxSpeed);
        } else {
            leftMaster.setPercentVoltage(leftVelocity);
            rightMaster.setPercentVoltage(rightVelocity);
        }
    }

    /**
     * Get the velocity of the left side of the drive.
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getLeftVel() { return leftMaster.getVelocity(); }

    /**
     * Get the velocity of the right side of the drive.
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getRightVel() { return rightMaster.getVelocity(); }

    /**
     * Get the position of the left side of the drive.
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getLeftPos() { return leftMaster.getPosition(); }

    /**
     * Get the position of the right side of the drive.
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getRightPos() { return rightMaster.getPosition(); }

    /**
     * Get the cached velocity of the left side of the drive.
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getLeftVelCached() { return leftMaster.getVelocityCached(); }

    /**
     * Get the cached velocity of the right side of the drive.
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getRightVelCached() { return rightMaster.getVelocityCached(); }

    /**
     * Get the cached position of the left side of the drive.
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getLeftPosCached() { return leftMaster.getPositionCached(); }

    /**
     * Get the cached position of the right side of the drive.
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getRightPosCached() { return rightMaster.getVelocityCached(); }
    
    /** Completely stop the robot by setting the voltage to each side to be 0. */
    public void fullStop() {
        setPercentVoltage(0, 0);
    }
  
    /**
     * Set the robot's heading.
     * @param heading The heading to set to, in degrees on [-180, 180].
     */
    public void setHeadingDegrees(final double heading) {
        ahrs.setAngleAdjustment(heading);
    }

    /**
     * Get the robot's angular velocity.
     * @return Angular velocity in degrees/sec
     */
    public double getAngularVel() {
        return -ahrs.getRate();
    }

    /**
     * Get the robot's angular displacement since being turned on.
     * @return Angular displacement in degrees.
     */
    public double getAngularDisplacement() {
        return -ahrs.getAngle();
    }

    /**
     * Get the pitch value.
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

    /**
     * Hold the current position.
     * @param leftPos the position to stop the left side at
     * @param rightPos the position to stop the right side at
     */
    public void holdPosition(final double leftPos, final double rightPos) {
        leftMaster.setSetpoint(leftPos);
        rightMaster.setSetpoint(rightPos);
    }

    public void setPercentVoltage(double leftPctVolts, double rightPctVolts) {
        leftMaster.setPercentVoltage(leftPctVolts);
        rightMaster.setPercentVoltage(rightPctVolts);
    } 

    /** Resets the position of the Talon to 0. */
    public void resetPosition() {
        leftMaster.resetPosition();
        rightMaster.resetPosition();
    }
           
    public void logPeriodic() {
        leftMaster.update();
        rightMaster.update();
    }

    public void updateDashboard()
    {
        SmartDashboard.putData("Field", m_field);
    }

    public void enableDriveTrain(boolean enable) {
        differentialDrive.setSafetyEnabled(enable);
        if (enable) {
            leftMaster.enable();
            rightMaster.enable();
        } else {
            leftMaster.disable();
            rightMaster.disable();
        }
    }

    public void enableBrakes(boolean enabled) {
        leftMaster.enableBrakes(enabled);
        rightMaster.enableBrakes(enabled);
    }

    public double getAverageEncoderDistance() {
        return (leftMaster.getPosition() + rightMaster.getPosition()) / 2.0;
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
        leftMaster.getTalon().configOpenloopRamp(rampTimeSeconds);
        rightMaster.getTalon().configOpenloopRamp(rampTimeSeconds);
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
        leftMaster.setTarget(meters);
        rightMaster.setTarget(meters);
    }
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
        differentialDrive.feed();
    }
}

