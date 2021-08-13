/**
 * Instrumentation Class that handles how telemetry from the Talon FX interacts
 * with Driverstation and Smart Dashboard.
 */
package frc.robot;

import frc.robot.wrappers.SmartMotor;
import frc.robot.wrappers.DriveGyro;

public class Instrumentation {
    /* Tracking variables for instrumentation */
    private static int _timesInProcessMotor = 0;
    private static int _timesInProcessGyro = 0;
    private static int _throttleCycles = 10;

    public static void setThrottle(int throttleCycles) {
        _throttleCycles = throttleCycles;
    }

    public static void ProcessMotor(SmartMotor motor) {
        if (++_timesInProcessMotor >= _throttleCycles) {
            motor.logPeriodic();
            _timesInProcessMotor = 0;
        }
    }

    public static void ProcessGyro(DriveGyro gyro) {
        if (++_timesInProcessGyro >= _throttleCycles) {
            gyro.logPeriodic();
        }
        _timesInProcessGyro = 0;
    }
}