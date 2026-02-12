package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Names;

public class Balls
{

	private static final RGB SAMPLE_PURPLE = new RGB(210, 55, 215);
	private static final RGB SAMPLE_GREEN = new RGB(40, 150, 45);
	private static final RGB SAMPLE_NONE = new RGB(22, 22, 22);
	
	private final RevColorSensorV3[] sensors = new RevColorSensorV3[3];
	private final Ball[] curBalls = new Ball[]{
			Ball.NONE,
			Ball.NONE,
			Ball.NONE
	};

	/**
	 * Initializes all 3 REV Color Sensor V3 devices.
	 */
	public Balls (HardwareMap hardwareMap)
	{

		sensors[0] = hardwareMap.get(RevColorSensorV3.class, Names.COLOR1);
		sensors[1] = hardwareMap.get(RevColorSensorV3.class, Names.COLOR2);
		sensors[2] = hardwareMap.get(RevColorSensorV3.class, Names.COLOR3);

		// Enable LEDs for all sensors
		for (int i = 0; i < 3; i++)
		{
			sensors[i].enableLed(true);
		}
	}

	/**
	 * Updates ball detection for all slots.
	 */
	public void update ()
	{

		for (int i = 0; i < 3; i++)
		{
			curBalls[i] = detectBall(sensors[i]);
		}
	}

	/**
	 * Returns the current 3-slot ball state.
	 */
	public Ball[] curBalls ()
	{

		return curBalls.clone();
	}

	/**
	 * Returns a single slot's ball state.
	 */
	public Ball getSlotStatus (int slot)
	{

		if (slot >= 0 && slot < 3)
		{
			return curBalls[slot];
		}
		return Ball.NONE;
	}

	/**
	 * Checks if all slots are empty.
	 */
	public boolean isEmpty ()
	{

		return curBalls[0] == Ball.NONE && curBalls[1] == Ball.NONE && curBalls[2] == Ball.NONE;
	}

	/**
	 * Checks if any slot contains a purple ball.
	 */
	public boolean hasPurple ()
	{

		for (Ball ball : curBalls)
		{
			if (ball == Ball.PURPLE) return true;
		}
		return false;
	}

	/**
	 * Checks if any slot contains a green ball.
	 */
	public boolean hasGreen ()
	{

		for (Ball ball : curBalls)
		{
			if (ball == Ball.GREEN) return true;
		}
		return false;
	}

	/**
	 * Counts how many balls are currently detected.
	 */
	public int countBalls ()
	{

		int count = 0;
		for (Ball ball : curBalls)
		{
			if (ball != Ball.NONE) count++;
		}
		return count;
	}

	private Ball detectBall (RevColorSensorV3 sensor)
	{

		RGB current = new RGB(sensor.red(), sensor.green(), sensor.blue());

		double noneDistance = rgbDistanceSquared(current, SAMPLE_NONE);
		double purpleDistance = rgbDistanceSquared(current, SAMPLE_PURPLE);
		double greenDistance = rgbDistanceSquared(current, SAMPLE_GREEN);

		if (noneDistance <= purpleDistance && noneDistance <= greenDistance)
		{
			return Ball.NONE;
		}
		if (purpleDistance <= greenDistance)
		{
			return Ball.PURPLE;
		}
		return Ball.GREEN;
	}

	private double rgbDistanceSquared (RGB a, RGB b)
	{

		int dr = a.red - b.red;
		int dg = a.green - b.green;
		int db = a.blue - b.blue;

		return dr * dr + dg * dg + db * db;
	}

	private static class RGB
	{
		final int red;
		final int green;
		final int blue;

		RGB (int red, int green, int blue)
		{

			this.red = red;
			this.green = green;
			this.blue = blue;
		}
	}
}
