package org.firstinspires.ftc.teamcode.Purple.Components.Motors;

public final class MotorUtil
{
	private MotorUtil ()
	{
	}

	/**
	 * Normalizes an array of motor powers so that no value exceeds 1.0 in magnitude.
	 *
	 * @param powers Array of motor powers
	 * @return Normalized array
	 */
	public static double[] normalizePowers (double[] powers)
	{
		double max = 0.0;
		for (double p : powers)
		{
			max = Math.max(max, Math.abs(p));
		}
		if (max > 1.0)
		{
			for (int i = 0; i < powers.length; i++)
			{
				powers[i] /= max;
			}
		}
		return powers;
	}

	/**
	 * Clamps a value between min and max.
	 *
	 * @param value The value to clamp.
	 * @param min   The minimum allowed value.
	 * @param max   The maximum allowed value.
	 * @return The clamped value.
	 */
	public static double clamp (double value, double min, double max)
	{
		return Math.max(min, Math.min(max, value));
	}

	/**
	 * Calculates the current RPM of a motor based on encoder ticks and elapsed time.
	 *
	 * @param currentPosition    The current encoder position.
	 * @param lastPosition       The previous encoder position.
	 * @param currentTime        The current time in milliseconds.
	 * @param lastTime           The previous time in milliseconds.
	 * @param ticksPerRevolution The number of encoder ticks per revolution of the motor.
	 * @return The calculated RPM.
	 */
	public static double calculateRPM (int currentPosition, int lastPosition, long currentTime, long lastTime, double ticksPerRevolution)
	{
		if (lastTime > 0 && currentTime > lastTime)
		{
			long deltaTime = currentTime - lastTime;
			int deltaPosition = currentPosition - lastPosition;

			// Calculate RPM: (ticks/ms) * (1000 ms/s) * (60 s/min) / (ticks/revolution)
			return (deltaPosition / (double) deltaTime) * 1000.0 * 60.0 / ticksPerRevolution;
		}
		return 0.0;
	}
}
