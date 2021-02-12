// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Hopper extends SubsystemBase {
    private final WPI_TalonSRX talonSRXEnhanced1 = new WPI_TalonSRX(3);

    public Hopper() {
        /* Factory default hardware to prevent unexpected behavior */
        talonSRXEnhanced1.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
        talonSRXEnhanced1.setInverted(false);
        talonSRXEnhanced1.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
        talonSRXEnhanced1.configNominalOutputForward(0, 30);
        talonSRXEnhanced1.configNominalOutputReverse(0, 30);
        talonSRXEnhanced1.configPeakOutputForward(0.75, 30);
        talonSRXEnhanced1.configPeakOutputReverse(-0.75, 30);
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void setMotorSpeed(double speed) {
        talonSRXEnhanced1.set(speed);
    }
}

