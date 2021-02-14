// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DriveTrain extends SubsystemBase {

    private final WPI_TalonFX talonFX1Left = new WPI_TalonFX(13);
    private final WPI_TalonFX talonFX2Right = new WPI_TalonFX(14);
    private final DifferentialDrive differentialDrive1 = new DifferentialDrive(talonFX1Left, talonFX2Right);
    private final WPI_TalonFX talonFX3Left = new WPI_TalonFX(15);
    private final WPI_TalonFX talonFX4Right = new WPI_TalonFX(12);

    public DriveTrain() {
        addChild("Differential Drive 1",differentialDrive1);
        differentialDrive1.setSafetyEnabled(true);
        differentialDrive1.setExpiration(0.1);
        differentialDrive1.setMaxOutput(0.75);

        talonFX3Left.follow(talonFX1Left);
        talonFX4Right.follow(talonFX2Right);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void drive(double speed, double rotation, boolean useSquares) {
        differentialDrive1.arcadeDrive(speed, rotation, useSquares);
    }
}

