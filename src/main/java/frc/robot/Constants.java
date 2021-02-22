package frc.robot;

public class Constants {

    public static final double kExtendHookSpeed = 0.25;
    public static final double kRetractHookSpeed = -0.25;

    public static final double kRetractWinchSpeed = -0.75;
    public static final double kReverseWinchSpeed = 0.75;

    public static final double kRunHopperSpeed = -0.75;
    public static final double kUnjamHopperSpeed = 0.75;

    public static final double kIntakeSpeed = -0.75;
    public static final double kUnjamIntakeSpeed = 0.75;    
    public static final double kRunShooterSpeed = 0.95;
    public static final double kSpinShooterSpeed = 0.64;

    public static final double kGearRatio = 10.71;
    public static final double kWheelRadiusInches = 3;
    public static final double kLengthChassisMeters = 1.0;
    public static final double kWidthChassisMeters = 0.8;

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final int k100msPerSecond = 10;
    public static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
}