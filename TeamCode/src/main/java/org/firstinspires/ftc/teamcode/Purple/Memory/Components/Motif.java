package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

/**
 * Represents a motif or pattern for memory storage.
 */
public class Motif
{
	private String pattern;

	/**
	 * Constructs an empty motif.
	 */
	public Motif ()
	{
		this.pattern = "";
	}

	/**
	 * Gets the motif pattern.
	 *
	 * @return The pattern string.
	 */
	public String getPattern ()
	{
		return pattern;
	}

	/**
	 * Sets the motif pattern.
	 *
	 * @param pattern The pattern string.
	 */
	public void setPattern (String pattern)
	{
		this.pattern = pattern;
	}
}
