package org.firstinspires.ftc.teamcode.Purple.Components.Sensors;

/**
 * Utility methods for color sensor processing
 */
public final class SensorUtil
{
	private SensorUtil ()
	{

	}

	/**
	 * Detects if a color is purple based on normalized RGB values
	 * Purple has high red and blue, low green
	 * @param red The normalized red value
	 * @param green The normalized green value
	 * @param blue The normalized blue value
	 * @param threshold The threshold for purple detection
	 * @return True if the color is purple, false otherwise
	 */
	public static boolean isPurple (double red, double green, double blue, double threshold)
	{
		// Purple typically has higher red and blue than green
		double purpleScore = (red + blue) / 2.0 - green;
		return purpleScore > threshold;
	}

	/**
	 * Detects if a color is green based on normalized RGB values
	 * Green has high green, low red and blue
	 * @param red The normalized red value
	 * @param green The normalized green value
	 * @param blue The normalized blue value
	 * @param threshold The threshold for green detection
	 * @return True if the color is green, false otherwise
	 */
	public static boolean isGreen (double red, double green, double blue, double threshold)
	{
		// Green typically has much higher green than red or blue
		double greenScore = green - (red + blue) / 2.0;
		return greenScore > threshold;
	}

	/**
	 * Detects ball color from normalized RGB values
	 * @param red The normalized red value
	 * @param green The normalized green value
	 * @param blue The normalized blue value
	 * @param purpleThreshold The threshold for purple detection
	 * @param greenThreshold The threshold for green detection
	 * @return 0 for none, 1 for purple, 2 for green
	 */
	public static int detectBallColor (double red, double green, double blue,
	                                   double purpleThreshold, double greenThreshold)
	{

		if (isPurple(red, green, blue, purpleThreshold))
		{
			return 1; // Purple
		} else if (isGreen(red, green, blue, greenThreshold))
		{
			return 2; // Green
		} else
		{
			return 0; // None or unknown
		}
	}

	/**
	 * Calculates overall brightness to avoid false positives in low light
	 * @param red The raw red value (0-255)
	 * @param green The raw green value (0-255)
	 * @param blue The raw blue value (0-255)
	 * @return The brightness (0-1)
	 */
	public static double getBrightness (int red, int green, int blue)
	{

		return (red + green + blue) / (3.0 * 255.0);
	}

	/**
	 * Smoothes sensor readings with exponential moving average
	 * @param current The current value
	 * @param previous The previous smoothed value
	 * @param alpha The smoothing factor
	 * @return The new smoothed value
	 */
	public static double smoothValue (double current, double previous, double alpha)
	{

		return alpha * current + (1 - alpha) * previous;
	}
}