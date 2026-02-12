package org.firstinspires.ftc.teamcode.Purple.Memory;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Balls;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Position;

/**
 * Shared robot state container for frequently read, changing values.
 */
public class PurpleMemory
{
	private final Balls balls;
	private final Position position;

	/**
	 * Constructs PurpleMemory.
	 */
	public PurpleMemory (HardwareMap hardwareMap)
	{
		balls = new Balls(hardwareMap);
		position = new Position(hardwareMap);
	}

	/**
	 * Updates all memory components.
	 */
	public void update ()
	{
		position.update();
		balls.update();
	}

	/**
	 * Gets current ball status array.
	 * 0 = none, 1 = purple, 2 = green
	 */
	public int[] balls ()
	{
		return balls.getBallStatus();
	}

	/**
	 * Gets the ball memory component.
	 */
	public Balls ballsState ()
	{
		return balls;
	}

	/**
	 * Gets the position memory component.
	 */
	public Position position ()
	{
		return position;
	}

	/**
	 * Backward-compatible alias for older call sites.
	 */
	public int[] Balls ()
	{
		return balls();
	}

	/**
	 * Backward-compatible alias for older call sites.
	 */
	public Balls getBallsObject ()
	{
		return ballsState();
	}

	/**
	 * Backward-compatible alias for older call sites.
	 */
	public Position getPositionObject ()
	{
		return position();
	}
}
