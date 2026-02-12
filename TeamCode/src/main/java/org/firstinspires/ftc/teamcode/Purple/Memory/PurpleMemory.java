package org.firstinspires.ftc.teamcode.Purple.Memory;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Ball;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Balls;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Motif;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Position;

/**
 * Shared robot state container for frequently read, changing values.
 */
public class PurpleMemory
{
	private final Balls balls;
	private final Motif motif;
	private final Position position;

	/**
	 * Constructs PurpleMemory.
	 */
	public PurpleMemory (HardwareMap hardwareMap)
	{
		balls = new Balls(hardwareMap);
		motif = new Motif();
		position = new Position(hardwareMap);
	}

	/**
	 * Updates all memory components.
	 */
	public void update ()
	{
		position.update();
		balls.update();
		motif.update();
	}

	/**
	 * Gets current ball states for the 3 slots.
	 */
	public Ball[] curBalls ()
	{
		return balls.curBalls();
	}

	/**
	 * Gets the currently detected motif.
	 */
	public Motif.Type curMotif ()
	{
		return motif.curMotif();
	}

	/**
	 * Gets the current odometry position.
	 */
	public Position curPos ()
	{
		return position;
	}
}
