package org.firstinspires.ftc.teamcode.Purple.Components.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Configuration for a color sensor
 */
public final class SensorConfig
{
	private final String name;
	private final ColorSensor sensor;

	// Thresholds for color detection (will be auto-tuned later)
	private double purpleThreshold = 0.6;
	private double greenThreshold = 0.6;

	/**
	 * Constructs a SensorConfig for a color sensor
	 * @param hardwareMap The hardware map
	 * @param name The name of the sensor in the hardware map
	 */
	public SensorConfig (HardwareMap hardwareMap, String name)
	{

		this.name = name;
		this.sensor = hardwareMap.get(ColorSensor.class, name);

		// Enable LED for better detection
		enableLED(true);
	}

	/**
	 * Gets the name of the sensor
	 * @return The name of the sensor
	 */
	public String getName ()
	{

		return name;
	}

	/**
	 * Gets the raw red value (0-255)
	 * @return The raw red value
	 */
	public int getRed ()
	{

		return sensor.red();
	}

	/**
	 * Gets the raw green value (0-255)
	 * @return The raw green value
	 */
	public int getGreen ()
	{

		return sensor.green();
	}

	/**
	 * Gets the raw blue value (0-255)
	 * @return The raw blue value
	 */
	public int getBlue ()
	{

		return sensor.blue();
	}

	/**
	 * Enables or disables the sensor's LED
	 * @param enable True to enable, false to disable
	 */
	public void enableLED (boolean enable)
	{

		sensor.enableLed(enable);
	}

	/**
	 * Gets the normalized color values (0-1)
	 * @return A double array containing the normalized RGB values
	 */
	public double[] getNormalizedRGB ()
	{

		int red = getRed();
		int green = getGreen();
		int blue = getBlue();
		int total = red + green + blue;

		if (total == 0) return new double[]{0, 0, 0};

		return new double[]{
				red / (double) total,
				green / (double) total,
				blue / (double) total
		};
	}

	/**
	 * Gets the purple threshold
	 * @return The purple threshold
	 */
	public double getPurpleThreshold ()
	{

		return purpleThreshold;
	}

	/**
	 * Sets the purple detection threshold
	 * @param threshold The new purple threshold
	 */
	public void setPurpleThreshold (double threshold)
	{

		this.purpleThreshold = threshold;
	}

	/**
	 * Gets the green threshold
	 * @return The green threshold
	 */
	public double getGreenThreshold ()
	{

		return greenThreshold;
	}

	/**
	 * Sets the green detection threshold
	 * @param threshold The new green threshold
	 */
	public void setGreenThreshold (double threshold)
	{

		this.greenThreshold = threshold;
	}
}