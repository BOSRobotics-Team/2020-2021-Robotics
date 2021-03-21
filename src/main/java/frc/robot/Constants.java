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
    public static final int kAuxLoopIdx = 1;
    public static final int kTimeoutMs = 30;
    public static final int k100msPerSecond = 10;

    public static final double kNeutralDeadband = 0.001;


    public final static int kSensorUnitsPerRotation = 2048;
	
	/* 	 * Number of rotations to drive when performing Distance Closed Loop  */
	public final static double kRotationsToTravel = 6;
	
	/**
	 * Empirically measure what the difference between encoders per 360'
	 * Drive the robot in clockwise rotations and measure the units per rotation.
	 * Drive the robot in counter clockwise rotations and measure the units per rotation.
	 * Take the average of the two.
	 */
	public final static int kEncoderUnitsPerRotation = 51711;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
	 * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
	public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
	public final static Gains kGains_Turning = new Gains( 0.2, 0.0,  0.0, 0.0,            200,  1.00 );
	public final static Gains kGains_Velocit = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
	public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 );
	
	/** ---- Flat constants, you should not need to change these ---- */
	/* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/* Firmware currently supports slots [0, 3] and can be used for either PID Set */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;

	public final static double kMaxSpeedMetersPerSecond = 3;
    public final static double kMaxAccelerationMetersPerSecondSquared = 3;
}