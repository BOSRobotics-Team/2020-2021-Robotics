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
import frc.robot.Constants;
import frc.robot.Convertor;

public class SmartMotor {

    private final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.

    final private WPI_TalonFX talon;
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
        talon = new WPI_TalonFX(deviceNumber);
        convertor = new Convertor(kCountsPerRev);

        talon.configFactoryDefault();

		/* Configure Sensor Source for Primary PID */
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        /* set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %) */
        talon.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

 		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
         */
        //talon.setSensorPhase(false);
        talon.setInverted(invert);

     	/* Set relevant frame periods to be at least as fast as periodic rate */
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, Constants.kTimeoutMs);
        
        /* Set the peak and nominal outputs */
		talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		talon.configPeakOutputForward(1, Constants.kTimeoutMs);
        talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        
		/* Set Motion Magic gains in slot0 - see documentation */
		talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		talon.config_kP(Constants.kSlotIdx, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
		talon.config_kI(Constants.kSlotIdx, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
		talon.config_kD(Constants.kSlotIdx, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
		talon.config_kF(Constants.kSlotIdx, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
		talon.config_IntegralZone(Constants.kSlotIdx, (int)Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
		talon.configMaxIntegralAccumulator(Constants.kSlotIdx, 1.0, Constants.kTimeoutMs);

   		/* Set acceleration and vcruise velocity - see documentation */
        talon.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
        talon.configMotionAcceleration(6000, Constants.kTimeoutMs);
    }

    public WPI_TalonFX getTalon() {
        return talon;
    }

    public void setClosedLoopGains(double kp, double ki, double kd, double kf, double iZone, double maxIntegral) {
		talon.config_kP(Constants.kSlotIdx, kp, Constants.kTimeoutMs);
		talon.config_kI(Constants.kSlotIdx, ki, Constants.kTimeoutMs);
		talon.config_kD(Constants.kSlotIdx, kd, Constants.kTimeoutMs);
		talon.config_kF(Constants.kSlotIdx, kf, Constants.kTimeoutMs);
		talon.config_IntegralZone(Constants.kSlotIdx, (int)iZone, Constants.kTimeoutMs);
		talon.configMaxIntegralAccumulator(Constants.kSlotIdx, maxIntegral, Constants.kTimeoutMs);
    }

    /**
     * Get the velocity of the drive.
     *
     * @return The signed velocity in meters per second, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getVelocity() {
        return convertor.nativeUnitsToVelocity(talon.getSelectedSensorVelocity(0));
    }

    /**
     * Get the position of the drive.
     *
     * @return The signed position in meters, or null if the drive doesn't have encoders.
     */
    @Nullable
    public Double getPosition() {
        return convertor.nativeUnitsToDistanceMeters(talon.getSelectedSensorPosition(0));
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
        talon.setVoltage(outputVolts);
    }
    public void set(double outputVolts) {
        talon.set(outputVolts);
    }
    public void setTarget(double meters) {
        int targetPos = convertor.distanceMetersToNativeUnits(meters);
        talon.set(TalonFXControlMode.MotionMagic, targetPos);
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
        talon.set(ControlMode.PercentOutput, pctVolts);
    }
    
    public void setVelocityUPS(final double velocity) {
        nativeSetpoint = convertor.velocityToNativeUnits(velocity);
        setpoint = velocity;

        talon.config_kF(0, 0, 0);
        talon.set(
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
        return talon.getMotorOutputVoltage();
    }
    public double getBatteryVoltage() {
        return talon.getBusVoltage();
    }
    public double getOutputCurrent() {
        return talon.getSupplyCurrent();
    }
    public String getControlMode() {
        return talon.getControlMode().name();
    }
    /** Resets the position of the Talon to 0. */
    public void resetPosition() {
        talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }
    public void enableBrakes(boolean enabled) {
        talon.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /** Updates all cached values with current ones. */
    public void update() {
        cachedVelocity = getVelocity();
        cachedPosition = getPosition();

        talon.getFaults(faults);
        if (faults.SensorOutOfPhase) {
            double leftVelUnitsPer100ms = talon.getSelectedSensorVelocity(0);
            System.out.println("sensor is out of phase: " + leftVelUnitsPer100ms);
        }
    }

    public void disable() {
        talon.disable();
    }
    public void enable() {
        talon.set(ControlMode.PercentOutput, 0);
    }
}
