package org.firstinspires.ftc.teamcode.Purple.Components.Motors;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simplified motor configuration class for basic power and velocity control
 * Uses MotorEx for all motors to ensure consistent velocity measurement
 */
public final class MotorConfig
{
	private final String name;
	private final Position position;
	private final MotorEx motor;
	private final double maxRPM;
	private final double cpr; // Counts per revolution

	private ControlMode controlMode = ControlMode.RAW_POWER;
	private double targetRPM = 0;

	private MotorConfig (Builder b)
	{

		this.name = b.name;
		this.position = b.position;
		this.maxRPM = b.maxRPM;
		this.cpr = b.cpr;

		// Always use MotorEx for consistent velocity measurement
		if (b.cpr != 0 && b.maxRPM != 0)
		{
			this.motor = new MotorEx(b.hardwareMap, b.name, b.cpr, b.maxRPM);
		} else
		{
			this.motor = new MotorEx(b.hardwareMap, b.name);
		}

		motor.setInverted(b.inverted);
		motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

		// Set initial control mode
		setControlMode(b.velocityEnabled ? ControlMode.VELOCITY_CONTROL : ControlMode.RAW_POWER);
	}

	/**
	 * Converts RPM to ticks per second
	 *
	 * @param rpm Revolutions per minute
	 * @return Ticks per second
	 */
	private double rpmToTps (double rpm)
	{

		if (cpr == 0) return 0;
		return rpm * (cpr / 60.0);
	}

	/**
	 * Converts ticks per second to RPM
	 *
	 * @param tps Ticks per second
	 * @return Revolutions per minute
	 */
	private double tpsToRpm (double tps)
	{

		if (cpr == 0) return 0;
		return tps * (60.0 / cpr);
	}

	/**
	 * Gets the current target RPM
	 *
	 * @return Target RPM value
	 */
	public double getTargetRPM ()
	{

		return targetRPM;
	}

	/**
	 * Sets target RPM using velocity control or falls back to power control
	 *
	 * @param rpm The target RPM to set
	 */
	public void setTargetRPM (double rpm)
	{

		this.targetRPM = rpm;

		if (controlMode == ControlMode.VELOCITY_CONTROL && maxRPM > 0)
		{
			// Convert RPM to ticks per second and set velocity
			double tps = rpmToTps(rpm);
			motor.setVelocity(tps);
		} else
		{
			// Fallback to power control
			double power = maxRPM > 0 ? rpm / maxRPM : Math.min(1.0, rpm / 1000.0);
			power = Math.max(-1.0, Math.min(1.0, power));
			setPower(power);
		}
	}

	/**
	 * Sets raw power to the motor (-1.0 to 1.0)
	 *
	 * @param power Power value between -1.0 and 1.0
	 */
	public void setPower (double power)
	{

		setControlMode(ControlMode.RAW_POWER);
		motor.set(power);
		this.targetRPM = power * maxRPM;
	}

	/**
	 * Gets the current RPM from motor velocity
	 *
	 * @return Current RPM value
	 */
	public double getCurrentRPM ()
	{
		// Use MotorEx's getVelocity() which returns ticks per second
		double tps = motor.getVelocity();
		return tpsToRpm(tps);
	}

	/**
	 * Gets the current control mode
	 *
	 * @return Current control mode
	 */
	public ControlMode getControlMode ()
	{

		return controlMode;
	}

	/**
	 * Sets the control mode for the motor
	 *
	 * @param mode Control mode to set
	 */
	public void setControlMode (ControlMode mode)
	{

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

	/**
	 * Gets the motor name
	 *
	 * @return Motor name
	 */
	public String getName ()
	{

		return name;
	}

	/**
	 * Gets the motor position
	 *
	 * @return Motor position
	 */
	public Position getPosition ()
	{

		return position;
	}

	/**
	 * Stops the motor
	 */
	public void stop ()
	{

		motor.stopMotor();
		targetRPM = 0;
	}

	/**
	 * Gets the maximum RPM capability
	 *
	 * @return Maximum RPM value
	 */
	public double getMaxRPM ()
	{

		return maxRPM;
	}

	/**
	 * Gets the counts per revolution (CPR)
	 *
	 * @return CPR value
	 */
	public double getCPR ()
	{

		return cpr;
	}

	/**
	 * Updates motor state - call in main loop for velocity control
	 */
	public void update ()
	{
		// MotorEx handles its own updates internally
		// This method is kept for interface consistency
	}

	// ---------------- Enums ----------------
	public enum Position
	{
		FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, EXPLOSHER, INTAKE, MIDTAKE
	}

	public enum ControlMode
	{
		RAW_POWER, VELOCITY_CONTROL, POSITION_CONTROL
	}

	// ---------------- Builder ----------------

	/**
	 * Builder class for MotorConfig
	 */
	public static class Builder
	{
		private final HardwareMap hardwareMap;
		private final String name;
		private final Position position;
		private final double maxRPM;
		private final double cpr;

		private boolean inverted = false;
		private Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
		private boolean velocityEnabled = true;

		/**
		 * Creates a new MotorConfig builder with CPR and max RPM
		 *
		 * @param hw       Hardware map
		 * @param name     Motor name
		 * @param position Motor position
		 * @param cpr      Counts per revolution
		 * @param maxRPM   Maximum RPM for velocity control
		 */
		public Builder (HardwareMap hw, String name, Position position, double cpr, double maxRPM)
		{

			this.hardwareMap = hw;
			this.name = name;
			this.position = position;
			this.cpr = cpr;
			this.maxRPM = maxRPM;
		}

		/**
		 * Creates a new MotorConfig builder without CPR and max RPM (power control only)
		 *
		 * @param hw       Hardware map
		 * @param name     Motor name
		 * @param position Motor position
		 */
		public Builder (HardwareMap hw, String name, Position position)
		{

			this(hw, name, position, 0, 0);
		}

		/**
		 * Sets motor direction as inverted
		 *
		 * @return Builder instance
		 */
		public Builder inverted ()
		{

			this.inverted = true;
			return this;
		}

		/**
		 * Sets zero power behavior
		 *
		 * @param zeroPowerBehavior Zero power behavior to set
		 * @return Builder instance
		 */
		public Builder zeroPowerBehavior (Motor.ZeroPowerBehavior zeroPowerBehavior)
		{

			this.zeroPowerBehavior = zeroPowerBehavior;
			return this;
		}

		/**
		 * Disables velocity control (uses raw power instead)
		 *
		 * @return Builder instance
		 */
		public Builder disableVelocityControl ()
		{

			this.velocityEnabled = false;
			return this;
		}

		/**
		 * Builds the MotorConfig instance
		 *
		 * @return Configured MotorConfig instance
		 */
		public MotorConfig build ()
		{

			return new MotorConfig(this);
		}
	}
}