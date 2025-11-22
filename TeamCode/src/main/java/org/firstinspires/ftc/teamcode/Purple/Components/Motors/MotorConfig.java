package org.firstinspires.ftc.teamcode.Purple.Components.Motors;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public final class MotorConfig
{

	private final String name;
	private final Position position;

	private final Motor motor;
	private final double cpr;
	private final double maxRPM;
	// -----------------------------
	//  Flags
	// -----------------------------
	private final boolean velocityEnabled;
	private final double velocitySmoothingAlpha = 0.15;
	private final double maxPowerStep = 0.06; // increments per loop
	// -----------------------------
	//  PID + Feedforward Coefficients (sane defaults)
	// -----------------------------

	// 0
	// -0.0013
	// -0.0004
	private double kP = 0.0013; // Increased from 0.0007
	private double kI = 0.0004;
	private double kD = 0.0;
	private double kV = 0.0013;
	private double kA = 0.0;
	// -----------------------------
	//  State Tracking
	// -----------------------------
	private MotorState state = MotorState.OFF;
	private ControlMode controlMode = ControlMode.RAW_POWER;
	private double targetRPM = 0;
	private double targetTPS = 0;
	// Velocity smoothing:
	private double velocityEMA = 0;
	// Power ramp rate limiting:
	private double lastPower = 0;

	private MotorConfig (Builder b)
	{

		this.name = b.name;
		this.position = b.position;
		this.maxRPM = b.maxRPM;
		this.velocityEnabled = b.velocityEnabled;

		if (b.useMotorEx)
		{
			this.motor = new MotorEx(b.hardwareMap, b.name);
		} else if (b.maxRPM != 0 && b.cpr != 0)
		{
			this.motor = new Motor(b.hardwareMap, b.name, b.cpr, b.maxRPM);
		} else
		{
			this.motor = new Motor(b.hardwareMap, b.name);
		}

		// prefer builder-specified cpr; fallback to motor provided value
		this.cpr = (b.cpr != 0) ? b.cpr : motor.getCPR();

		motor.setInverted(b.inverted);
		motor.setZeroPowerBehavior(b.zeroPowerBehavior);

		if (velocityEnabled)
		{
			applyPID();
			applyFeedforward();
			setControlMode(ControlMode.VELOCITY_CONTROL);
		} else
		{
			// ensure raw power mode when velocity disabled
			setControlMode(ControlMode.RAW_POWER);
		}
	}

	// ---------------- PID/FF ----------------
	private void applyPID ()
	{

		motor.setVeloCoefficients(kP, kI, kD);
		motor.setPositionCoefficient(1.0);
	}

	private void applyFeedforward ()
	{

		motor.setFeedforwardCoefficients(kV, kA);
	}

	// ---------------- Unit conversions ----------------
	private double rpmToTps (double rpm)
	{

		if (cpr == 0) return 0;
		return rpm * (cpr / 60.0);
	}

	private double tpsToRpm (double tps)
	{

		if (cpr == 0) return 0;
		return tps * (60.0 / cpr);
	}

	// ---------------- Public API ----------------

	public double getTargetRPM ()
	{
		return targetRPM;
	}

	/**
	 * Set target RPM. If this MotorConfig has velocity control enabled it sends a ticks/sec target.
	 * If velocity control is disabled this converts rpm->fraction of configured maxRPM and sets raw power.
	 */
	public void setTargetRPM (double rpm)
	{

        // if (rpm < 0) rpm = 0;
		this.targetRPM = rpm;

		if (velocityEnabled && maxRPM > 0)
		{
			if (controlMode != ControlMode.VELOCITY_CONTROL)
			{
				setControlMode(ControlMode.VELOCITY_CONTROL);
			}
			this.targetTPS = rpmToTps(rpm);
			motor.set(targetTPS);

			// Debugging: Log the target TPS and RPM
			DebugUtil.logAdd("MotorConfig [" + name + "] Target RPM: " + rpm);
			DebugUtil.logAdd("MotorConfig [" + name + "] Target TPS: " + targetTPS);

			state = (rpm != 0) ? MotorState.ON : MotorState.OFF;
		} else
		{
			// Fallback: convert to raw power fraction using maxRPM if available
			double power = 0;
			if (maxRPM > 0)
			{
				power = rpm / maxRPM; // simple linear mapping
			} else
			{
				// no maxRPM info — conservatively map small RPM to small power
				power = Math.min(1.0, rpm / 1000.0);
			}
			power = Math.max(-1.0, Math.min(1.0, power));
			setPower(power);
		}
	}

	public void setPower (double pwr)
	{

		setControlMode(ControlMode.RAW_POWER);

		// Smooth ramp to avoid current spikes
		double diff = pwr - lastPower;
		if (Math.abs(diff) > maxPowerStep)
		{
			pwr = lastPower + Math.signum(diff) * maxPowerStep;
		}

		lastPower = pwr;
		motor.set(pwr);
		state = (pwr != 0) ? MotorState.ON : MotorState.OFF;
	}

	/**
	 * Periodic update to maintain EMA of velocity (call from your OpMode loop).
	 */
	public void update ()
	{
		// Get the raw ticks per second from the motor
		double rawTps = motor.getCorrectedVelocity();

		// Debugging: Log the raw ticks per second and motor power
		DebugUtil.logAdd("MotorConfig [" + name + "] Raw TPS: " + rawTps);
		DebugUtil.logAdd("MotorConfig [" + name + "] Motor Power: " + lastPower);

		// Update the velocity EMA (Exponential Moving Average)
		velocityEMA = velocityEMA + velocitySmoothingAlpha * (rawTps - velocityEMA);
	}

	public double getCurrentRPM ()
	{

		return tpsToRpm(velocityEMA);
	}

	public MotorState getState ()
	{

		return state;
	}

	public ControlMode getControlMode ()
	{

		return controlMode;
	}

	public void setControlMode (ControlMode mode)
	{
		// Do not allow velocity mode if velocityEnabled == false
		if (mode == ControlMode.VELOCITY_CONTROL && !velocityEnabled)
		{
			this.controlMode = ControlMode.RAW_POWER;
			motor.setRunMode(Motor.RunMode.RawPower);
			return;
		}

		this.controlMode = mode;
		switch (mode)
		{
			case VELOCITY_CONTROL:
				motor.setRunMode(Motor.RunMode.VelocityControl);
				break;
			case POSITION_CONTROL:
				motor.setRunMode(Motor.RunMode.PositionControl);
				break;
			case RAW_POWER:
			default:
				motor.setRunMode(Motor.RunMode.RawPower);
				break;
		}
	}

	public String getName ()
	{

		return name;
	}

	public Position getPosition ()
	{

		return position;
	}

	public void stop ()
	{

		motor.stopMotor();
		state = MotorState.OFF;
		targetRPM = 0;
		targetTPS = 0;
	}

	public double getMaxRPM ()
	{

		return maxRPM;
	}

	// ---------------- Enums ----------------
	public enum Position
	{
		FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, EXPLOSHER, INTAKE, MIDTAKE
	}

	public enum MotorState
	{ON, OFF}

	public enum ControlMode
	{RAW_POWER, VELOCITY_CONTROL, POSITION_CONTROL}

	// ---------------- Builder ----------------
	public static class Builder
	{
		private final HardwareMap hardwareMap;
		private final String name;
		private final Position position;

		private final double maxRPM;
		private final double cpr;

		private boolean inverted = false;
		private Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
		private boolean useMotorEx = false;

		// new: allow disabling velocity control for drive motors, defaults to true (enabled)
		private boolean velocityEnabled = true;

		public Builder (HardwareMap hw, String name, Position position, double cpr, double maxRPM)
		{

			this.hardwareMap = hw;
			this.name = name;
			this.position = position;
			this.cpr = cpr;
			this.maxRPM = maxRPM;
		}

		public Builder (HardwareMap hw, String name, Position position)
		{

			this(hw, name, position, 0, 0);
		}

		public Builder inverted ()
		{

			this.inverted = true;
			return this;
		}

		public Builder zeroPowerBehavior (Motor.ZeroPowerBehavior zeroPowerBehavior)
		{

			this.zeroPowerBehavior = zeroPowerBehavior;
			return this;
		}

		public Builder useMotorEx ()
		{

			this.useMotorEx = true;
			return this;
		}

		/**
		 * Disable velocity control for this motor. Useful for drive motors where you want raw power.
		 */
		public Builder disableVelocityControl ()
		{

			this.velocityEnabled = false;
			return this;
		}

		public MotorConfig build ()
		{

			return new MotorConfig(this);
		}
	}
}
