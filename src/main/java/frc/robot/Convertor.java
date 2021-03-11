package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public class Convertor {
    public final int mCountsPerRev;

    public Convertor( int countsPerRev ) {
        mCountsPerRev = countsPerRev;
    }

    public int distanceMetersToNativeUnits(double positionMeters) {
        double wheelRotations = positionMeters / (2. * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        double motorRotations = wheelRotations * Constants.kGearRatio;
        int sensorCounts = (int)(motorRotations * mCountsPerRev);
        return sensorCounts;
    }
    public double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = (double)sensorCounts / mCountsPerRev;
        double wheelRotations = motorRotations / Constants.kGearRatio;
        double positionMeters = wheelRotations * (2. * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        return positionMeters;
    }

    public int velocityToNativeUnits(double velocityMetersPerSecond) {
        double wheelRotationsPerSecond = velocityMetersPerSecond / (2. * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / Constants.k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * mCountsPerRev);
        return sensorCountsPer100ms;
    }    
    public double nativeUnitsToVelocity(double sensorCountsPer100ms) {
        double motorRotationsPer100ms = (double)sensorCountsPer100ms / mCountsPerRev;
        double motorRotationsPerSecond = motorRotationsPer100ms * Constants.k100msPerSecond;
        double wheelRotationsPerSecond = motorRotationsPerSecond / Constants.kGearRatio;
        double velocityMetersPerSecond = wheelRotationsPerSecond * (2. * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
        return velocityMetersPerSecond;
    }  
}
