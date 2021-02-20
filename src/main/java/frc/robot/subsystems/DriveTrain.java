// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;


public class DriveTrain extends SubsystemBase {

    private final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
    private final double kGearRatio = 10.71;
    private final double kWheelRadiusInches = 3;
    private final int k100msPerSecond = 10;

    private final AHRS navx_device = new AHRS();

    private final WPI_TalonFX talonFX1Left = new WPI_TalonFX(13);
    private final WPI_TalonFX talonFX2Right = new WPI_TalonFX(14);
    private final WPI_TalonFX talonFX3Left = new WPI_TalonFX(15);
    private final WPI_TalonFX talonFX4Right = new WPI_TalonFX(12);

    private final DifferentialDrive differentialDrive1 = new DifferentialDrive(talonFX1Left, talonFX2Right);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(navx_device.getRotation2d());
    private final Field2d m_field = new Field2d();

    public DriveTrain() {
        addChild("Differential Drive 1",differentialDrive1);
        differentialDrive1.setSafetyEnabled(true);
        differentialDrive1.setExpiration(0.1);
        differentialDrive1.setMaxOutput(0.75);

        talonFX3Left.follow(talonFX1Left);
        talonFX4Right.follow(talonFX2Right);

        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(navx_device.getRotation2d(),
                      nativeUnitsToDistanceMeters(talonFX1Left.getSelectedSensorPosition()),
                      nativeUnitsToDistanceMeters(talonFX2Right.getSelectedSensorPosition()));
        m_field.setRobotPose(m_odometry.getPoseMeters());
//        m_odometry.update(navx_device.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, navx_device.getRotation2d());
    }

    public void resetEncoders() {
        talonFX1Left.setSelectedSensorPosition(0);
        talonFX2Right.setSelectedSensorPosition(0);
    }

    public double getAverageEncoderDistance() {
        return (nativeUnitsToDistanceMeters(talonFX1Left.getSelectedSensorPosition()) +
                nativeUnitsToDistanceMeters(talonFX2Right.getSelectedSensorPosition())) / 2.0;
    }

    public double getHeading() {
        return navx_device.getRotation2d().getDegrees();
    }
    public double getTurnRate() {
        return -navx_device.getRate();
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void drive(double speed, double rotation, boolean useSquares) {
        differentialDrive1.arcadeDrive(speed, rotation, useSquares);
    }
    
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        differentialDrive1.tankDrive(leftVolts, rightVolts);
    }
    private int distanceToNativeUnits(double positionMeters) {
        double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
        double motorRotations = wheelRotations * kGearRatio;
        int sensorCounts = (int)(motorRotations * kCountsPerRev);
        return sensorCounts;
    }
    
    private int velocityToNativeUnits(double velocityMetersPerSecond) {
        double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
        double motorRotationsPerSecond = wheelRotationsPerSecond * kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
        return sensorCountsPer100ms;
    }
    
    private double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = (double)sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / kGearRatio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
        return positionMeters;
    }
}

