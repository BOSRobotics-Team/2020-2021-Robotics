package frc.robot.wrappers;

import javax.annotation.Nullable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Convertor;

public class SmartMotor extends WPI_TalonFX {

    private final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.

    final private Convertor convertor;

    private SimpleMotorFeedforward feedForwardCalculator = new SimpleMotorFeedforward(0, 0, 0);
    private final Faults faults = new Faults();

    /** Cached values for various sensor readings. */
    private double cachedVelocity = Double.NaN;
    private double cachedPosition = Double.NaN;

    /** The most recently set setpoint. */
    private double setpoint;

    /** The setpoint in native units. Field to avoid garbage collection. */
    private double nativeSetpoint;

    public SmartMotor( int deviceNumber, TalonFXInvertType invert ) {
        super(deviceNumber);
        convertor = new Convertor(kCountsPerRev);

        this.configFactoryDefault();

		/* Configure Sensor Source for Primary PID */
        this.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
        this.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

 		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
         */
        //this.setSensorPhase(false);
        this.setInverted(invert);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
        this.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        this.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, Constants.kTimeoutMs);
        
        /* Set the peak and nominal outputs */
		this.configNominalOutputForward(0, Constants.kTimeoutMs);
		this.configNominalOutputReverse(0, Constants.kTimeoutMs);
		this.configPeakOutputForward(1, Constants.kTimeoutMs);
        this.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        
		/* Set Motion Magic gains in slot0 - see documentation */
		this.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        this.setClosedLoopGains(Constants.kSlotIdx, 
            Constants.kGains_Distanc.kP, 
            Constants.kGains_Distanc.kI, 
            Constants.kGains_Distanc.kD, 
            Constants.kGains_Distanc.kF, 
            Constants.kGains_Distanc.kIzone, 
            1.0);

   		/* Set acceleration and vcruise velocity - see documentation */
        this.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
        this.configMotionAcceleration(6000, Constants.kTimeoutMs);
    }

    public void setClosedLoopGains(int slot, double kp, double ki, double kd, double kf, double iZone, double maxIntegral) {
		this.config_kP(Constants.kSlotIdx, kp, Constants.kTimeoutMs);
		this.config_kI(Constants.kSlotIdx, ki, Constants.kTimeoutMs);
		this.config_kD(Constants.kSlotIdx, kd, Constants.kTimeoutMs);
		this.config_kF(Constants.kSlotIdx, kf, Constants.kTimeoutMs);
		this.config_IntegralZone(Constants.kSlotIdx, (int)iZone, Constants.kTimeoutMs);
		this.configMaxIntegralAccumulator(Constants.kSlotIdx, maxIntegral, Constants.kTimeoutMs);
    }

    /**
     * Get the velocity of the drive.
     *
     * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getVelocity() {
        return convertor.nativeUnitsToVelocity(this.getSelectedSensorVelocity(0));
    }

    /**
     * Get the position of the drive.
     *
     * @return The signed position in meters, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getPosition() {
        return convertor.nativeUnitsToDistanceMeters(this.getSelectedSensorPosition(0));
    }

    /**
     * Get the cached velocity of the drive.
     *
     * @return The signed velocity in feet per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getVelocityCached() {
        return cachedVelocity;
    }

    /**
     * Get the cached position of the drive.
     *
     * @return The signed position in feet, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getPositionCached() {
        return cachedPosition;
    }

    public void setVoltage(double outputVolts) {
        this.setVoltage(outputVolts);
    }
    public void set(double outputVolts) {
        this.set(outputVolts);
    }
    public void setTarget(double meters) {
        int targetPos = convertor.distanceMetersToNativeUnits(meters);
        this.set(TalonFXControlMode.MotionMagic, targetPos);
    }

    public void setPercentVoltage(double pctVolts) {
    // Warn the user if they're setting Vbus to a number that's outside the range of values.
        if (Math.abs(pctVolts) > 1.0) {
            Shuffleboard.addEventMarker(
                "WARNING: YOU ARE CLIPPING MAX PERCENT VBUS AT L:" + pctVolts,
                getClass().getSimpleName(),
                EventImportance.kNormal);

                pctVolts = Math.signum(pctVolts);
        }
        setpoint = pctVolts;
        this.set(ControlMode.PercentOutput, pctVolts);
    }
    
    public void setVelocityUPS(final double velocity) {
        nativeSetpoint = convertor.velocityToNativeUnits(velocity);
        setpoint = velocity;

        this.config_kF(0, 0, 0);
        this.set(
            ControlMode.Velocity,
            nativeSetpoint,
            DemandType.ArbitraryFeedForward,
            feedForwardCalculator.calculate(velocity) / 12.);
    }

    public double getSetpoint() {
        return setpoint;
    }
    public void setSetpoint(double pt) {
        setpoint = pt;
    }
    public double getOutputVoltage() {
        return this.getMotorOutputVoltage();
    }
    public double getBatteryVoltage() {
        return this.getBusVoltage();
    }
    public double getOutputCurrent() {
        return this.getSupplyCurrent();
    }
    /** Resets the position of the Talon to 0. */
    public void resetPosition() {
        this.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }
    public void enableBrakes(boolean enabled) {
        this.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /** Updates all cached values with current ones. */
    public void update() {
        cachedVelocity = getVelocity();
        cachedPosition = getPosition();

        this.getFaults(faults);
        if (faults.SensorOutOfPhase) {
            double leftVelUnitsPer100ms = this.getSelectedSensorVelocity(0);
            System.out.println("sensor is out of phase: " + leftVelUnitsPer100ms);
        }
    }

    public void logPeriodic() {
        String name = this.getName() + " ";

        SmartDashboard.putNumber(name + " cachePos: ", cachedPosition);
        SmartDashboard.putNumber(name + " cacheVel: ", cachedVelocity);

        /* Smart dash plots */
        SmartDashboard.putNumber(name + "SensorVel", this.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
        SmartDashboard.putNumber(name + "SensorPos", this.getSelectedSensorPosition(Constants.kPIDLoopIdx));
        SmartDashboard.putNumber(name + "MotorOutputPercent", this.getMotorOutputPercent());
        SmartDashboard.putNumber(name + "ClosedLoopError", this.getClosedLoopError(Constants.kPIDLoopIdx));
        SmartDashboard.putString(name + "ControlMode", this.getControlMode().toString());
        
        /* Print the Active Trajectory Point Motion Magic is servoing towards */
        SmartDashboard.putNumber(name + "ClosedLoopTarget", this.getClosedLoopTarget(Constants.kPIDLoopIdx));
        SmartDashboard.putNumber(name + "ActTrajVelocity", this.getActiveTrajectoryVelocity());
        SmartDashboard.putNumber(name + "ActTrajPosition", this.getActiveTrajectoryPosition());        
    }

    public void disable() {
        this.disable();
    }
    public void enable() {
        this.set(ControlMode.PercentOutput, 0);
    }
}
