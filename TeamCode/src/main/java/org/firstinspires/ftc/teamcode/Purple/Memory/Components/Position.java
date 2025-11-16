package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

/**
 * Represents a position with x, y coordinates and heading.
 */
public class Position
{
	private double x, y, heading;

	/**
	 * Constructs a position at (0, 0, 0).
	 */
	public Position ()
	{
		this.x = 0;
		this.y = 0;
		this.heading = 0;
	}

	/**
	 * Constructs a position with specified coordinates and heading.
	 *
	 * @param x       The x coordinate.
	 * @param y       The y coordinate.
	 * @param heading The heading in radians.
	 */
	public Position (double x, double y, double heading)
	{
		this.x = x;
		this.y = y;
		this.heading = heading;
	}

	/**
	 * Gets the x coordinate.
	 *
	 * @return The x value.
	 */
	public double getX ()
	{
		return x;
	}

	/**
	 * Sets the x coordinate.
	 *
	 * @param x The new x value.
	 */
	public void setX (double x)
	{
		this.x = x;
	}

	/**
	 * Gets the y coordinate.
	 *
	 * @return The y value.
	 */
	public double getY ()
	{
		return y;
	}

	/**
	 * Sets the y coordinate.
	 *
	 * @param y The new y value.
	 */
	public void setY (double y)
	{
		this.y = y;
	}

	/**
	 * Gets the heading in radians.
	 *
	 * @return The heading value.
	 */
	public double getHeading ()
	{
		return heading;
	}

	/**
	 * Sets the heading in radians.
	 *
	 * @param heading The new heading value.
	 */
	public void setHeading (double heading)
	{
		this.heading = heading;
	}
}
