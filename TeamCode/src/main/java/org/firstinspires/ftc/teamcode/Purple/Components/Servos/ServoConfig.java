package org.firstinspires.ftc.teamcode.Purple.Components.Servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class ServoConfig
{
	private static final double POSITION_EPSILON = 1e-4;

	private final String name;
	private final Servo servo;
	private final double minPosition;
	private final double maxPosition;
	private ServoState state = ServoState.OFF;
	public static double zeroOffset;

	private ServoConfig (Builder b)
	{
		if (b.minPosition > b.maxPosition)
		{
			throw new IllegalArgumentException("Servo minPosition cannot be greater than maxPosition");
		}

		this.name = b.name;
		this.minPosition = b.minPosition;
		this.maxPosition = b.maxPosition;
		this.servo = b.hardwareMap.get(Servo.class, b.name);
		this.servo.setDirection(b.direction);
	}

	/**
	 * Gets the name of the servo.
	 *
	 * @return The servo name.
	 */
	public String getName ()
	{
		return name;
	}

	/**
	 * Gets the minimum allowed position for the servo.
	 *
	 * @return The minimum position.
	 */
	public double getMinPosition ()
	{
		return minPosition;
	}

	/**
	 * Gets the maximum allowed position for the servo.
	 *
	 * @return The maximum position.
	 */
	public double getMaxPosition ()
	{
		return maxPosition;
	}

	/**
	 * Gets the wrapped servo hardware object.
	 *
	 * @return Wrapped servo.
	 */
	public Servo getServo ()
	{
		return servo;
	}

	/**
	 * Gets the current servo position.
	 *
	 * @return Current servo position.
	 */
	public double getPosition ()
	{
		return servo.getPosition();
	}

	/**
	 * Sets the servo position (clamped to configured range).
	 *
	 * @param position Requested position.
	 */
	public void setPosition (double position)
	{
		servo.setPosition(clamp(position));
	}

	/**
	 * Gets the current state of the servo (ON/OFF).
	 *
	 * @return The servo state.
	 */
	public ServoState getState ()
	{
		return state;
	}

	/**
	 * Sets the state of the servo (ON/OFF).
	 *
	 * @param state The desired servo state.
	 */
	public void setState (ServoState state)
	{
		this.state = state;
	}

	/**
	 * Clamps a position to the allowed range for this servo.
	 *
	 * @param position The position to clamp.
	 * @return The clamped position.
	 */
	public double clamp (double position)
	{
		return Math.max(minPosition, Math.min(maxPosition, position));
	}

	/**
	 * Represents the state of the servo (ON/OFF).
	 */
	public enum ServoState
	{
		ON, OFF
	}

	public static class Builder
	{
		private final HardwareMap hardwareMap;
		private final String name;

		private double minPosition = 0.0;
		private double maxPosition = 1.0;
		private Servo.Direction direction = Servo.Direction.FORWARD;

		public Builder (HardwareMap hardwareMap, String name)
		{
			this.hardwareMap = hardwareMap;
			this.name = name;
		}

		public Builder setMinPosition (double minPosition)
		{
			this.minPosition = minPosition;
			return this;
		}

		public Builder setMaxPosition (double maxPosition)
		{
			this.maxPosition = maxPosition;
			return this;
		}

		public Builder setRange (double minPosition, double maxPosition)
		{
			this.minPosition = minPosition;
			this.maxPosition = maxPosition;
			return this;
		}

		public Builder reversed ()
		{
			this.direction = Servo.Direction.REVERSE;
			return this;
		}

		public Builder direction (Servo.Direction direction)
		{
			this.direction = direction;
			return this;
		}

		public ServoConfig build ()
		{
			return new ServoConfig(this);
		}
	}
}
