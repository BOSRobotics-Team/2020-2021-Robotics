// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Shooter extends SubsystemBase {
    private final WPI_VictorSPX victorSPX1 = new WPI_VictorSPX(1);
    private final WPI_TalonSRX talonSRXEnhanced1 = new WPI_TalonSRX(0);

    public Shooter() {
        victorSPX1.follow(talonSRXEnhanced1);

        //talonSRXEnhanced1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                                //Constants.kPIDLoopIdx,
                                                 //Constants.kTimeoutMs);
        /*Factory default hardware to prevent unexpected behavior */
        //talonSRXEnhanced1.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
        //talonSRXEnhanced1.setSensorPhase(true);
        //talonSRXEnhanced1.setInverted(false);
        //talonSRXEnhanced1.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
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
    public void setmotorspeed(double speed) {
        talonSRXEnhanced1.set(speed);
    }
}

