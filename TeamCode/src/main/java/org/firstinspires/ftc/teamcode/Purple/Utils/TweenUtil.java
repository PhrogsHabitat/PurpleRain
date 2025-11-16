package org.firstinspires.ftc.teamcode.Purple.Utils;

public class TweenUtil
{
	/**
	 * Linear interpolation between a and b by t (0.0 to 1.0)
	 *
	 * @param a The start value.
	 * @param b The end value.
	 * @param t The interpolation factor (0.0 to 1.0).
	 * @return The interpolated value.
	 */
	public static double lerp (double a, double b, double t)
	{
		return a + (b - a) * t;
	}

	/**
	 * Smoothstep interpolation between a and b by t (0.0 to 1.0)
	 *
	 * @param a The start value.
	 * @param b The end value.
	 * @param t The interpolation factor (0.0 to 1.0).
	 * @return The smoothstep interpolated value.
	 */
	public static double smoothStep (double a, double b, double t)
	{
		t = Math.max(0, Math.min(1, t));
		t = t * t * (3 - 2 * t);
		return a + (b - a) * t;
	}
}
