// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Hook extends SubsystemBase {
    private final WPI_VictorSPX victorSPX1 = new WPI_VictorSPX(2);
    
    public Hook() {
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
    public void runHook(double speed) {
        victorSPX1.set(speed);
    }
}

