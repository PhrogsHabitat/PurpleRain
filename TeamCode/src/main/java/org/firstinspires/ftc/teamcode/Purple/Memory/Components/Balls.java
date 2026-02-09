package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Sensors.SensorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Sensors.SensorUtil;

public class Balls
{
	private static final double SMOOTHING_ALPHA = 0.3;
	private static final double MIN_BRIGHTNESS = 0.1;
	private final SensorConfig[] sensors = new SensorConfig[3];
	private final double[] smoothedRed = new double[3];
	private final double[] smoothedGreen = new double[3];
	private final double[] smoothedBlue = new double[3];
	public int[] ballStatus = new int[3]; // 0 = none, 1 = purple, 2 = green

	/**
	 * Initializes all 3 color sensors
	 */
	public Balls (HardwareMap hardwareMap)
	{
		// Initialize sensors (update names based on your hardware config)
		sensors[0] = new SensorConfig(hardwareMap, "colorSensor1");
		sensors[1] = new SensorConfig(hardwareMap, "colorSensor2");
		sensors[2] = new SensorConfig(hardwareMap, "colorSensor3");

		// Enable LEDs for all sensors
		for (SensorConfig sensor : sensors)
		{
			sensor.enableLED(true);
		}

		// Initialize arrays
		for (int i = 0; i < 3; i++)
		{
			ballStatus[i] = 0;
			smoothedRed[i] = smoothedGreen[i] = smoothedBlue[i] = 0;
		}
	}

	/**
	 * Updates ball detection for all slots
	 */
	public void update ()
	{

		for (int i = 0; i < 3; i++)
		{
			SensorConfig sensor = sensors[i];

			// Get raw RGB values
			int red = sensor.getRed();
			int green = sensor.getGreen();
			int blue = sensor.getBlue();

			// Calculate brightness to avoid false positives
			double brightness = SensorUtil.getBrightness(red, green, blue);

			if (brightness < MIN_BRIGHTNESS)
			{
				// Too dark, assume no ball
				ballStatus[i] = 0;
				continue;
			}

			// Smooth readings to reduce noise
			double[] normalized = sensor.getNormalizedRGB();
			smoothedRed[i] = SensorUtil.smoothValue(normalized[0], smoothedRed[i], SMOOTHING_ALPHA);
			smoothedGreen[i] = SensorUtil.smoothValue(normalized[1], smoothedGreen[i], SMOOTHING_ALPHA);
			smoothedBlue[i] = SensorUtil.smoothValue(normalized[2], smoothedBlue[i], SMOOTHING_ALPHA);

			// Detect ball color
			ballStatus[i] = SensorUtil.detectBallColor(
					smoothedRed[i], smoothedGreen[i], smoothedBlue[i],
					sensor.getPurpleThreshold(), sensor.getGreenThreshold()
			);
		}
	}

	/**
	 * Gets the array of current ball status
	 */
	public int[] getBallStatus ()
	{

		return ballStatus;
	}

	/**
	 * Gets status of a specific slot
	 */
	public int getSlotStatus (int slot)
	{

		if (slot >= 0 && slot < 3)
		{
			return ballStatus[slot];
		}
		return 0;
	}

	/**
	 * Checks if all slots are empty
	 */
	public boolean isEmpty ()
	{

		return ballStatus[0] == 0 && ballStatus[1] == 0 && ballStatus[2] == 0;
	}

	/**
	 * Checks if any slot contains a purple ball
	 */
	public boolean hasPurple ()
	{

		for (int status : ballStatus)
		{
			if (status == 1) return true;
		}
		return false;
	}

	/**
	 * Checks if any slot contains a green ball
	 */
	public boolean hasGreen ()
	{

		for (int status : ballStatus)
		{
			if (status == 2) return true;
		}
		return false;
	}

	/**
	 * Counts how many balls are currently detected
	 */
	public int countBalls ()
	{

		int count = 0;
		for (int status : ballStatus)
		{
			if (status != 0) count++;
		}
		return count;
	}
}