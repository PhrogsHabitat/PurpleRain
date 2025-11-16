package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

/**
 * Tracks the number of balls for memory or game logic.
 */
public class Balls
{
	private int count;

	/**
	 * Constructs a Balls tracker with zero balls.
	 */
	public Balls ()
	{
		this.count = 0;
	}

	/**
	 * Gets the current ball count.
	 *
	 * @return The ball count.
	 */
	public int getCount ()
	{
		return count;
	}

	/**
	 * Sets the ball count.
	 *
	 * @param count The new ball count.
	 */
	public void setCount (int count)
	{
		this.count = count;
	}

	/**
	 * Increments the ball count by one.
	 */
	public void increment ()
	{
		count++;
	}

	/**
	 * Decrements the ball count by one, not going below zero.
	 */
	public void decrement ()
	{
		if (count > 0) count--;
	}
}
