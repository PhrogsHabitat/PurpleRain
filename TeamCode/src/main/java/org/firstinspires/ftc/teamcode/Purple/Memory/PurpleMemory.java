package org.firstinspires.ftc.teamcode.Purple.Memory;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Ball;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Balls;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Motif;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Persist;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Position;

/**
 * Shared robot state container for frequently read, changing values.
 */
public class PurpleMemory
{
	public static PurpleMemory Instance;

	private final Balls balls;
	private final Motif motif;
	private final Persist persist;
	private final Position position;

	/**
	 * Constructs PurpleMemory.
	 */
	public PurpleMemory (HardwareMap hardwareMap)
	{

		this(hardwareMap, null);
	}

	/**
	 * Constructs PurpleMemory.
	 */
	public PurpleMemory (HardwareMap hardwareMap, Follower follower)
	{

		balls = new Balls(hardwareMap);
		motif = new Motif();
		persist = new Persist(hardwareMap);
		// Keep a lightweight position object only for API compatibility.
		// Live drivetrain localization should come from Pedro follower pose.
		position = follower == null ? new Position() : new Position(follower);
		Instance = this;
	}

	/**
	 * Recreates the shared memory instance for the current OpMode run.
	 */
	public static PurpleMemory initialize (HardwareMap hardwareMap)
	{

		Instance = new PurpleMemory(hardwareMap, null);
		return Instance;
	}

	/**
	 * Recreates the shared memory instance for the current OpMode run.
	 */
	public static PurpleMemory initialize (HardwareMap hardwareMap, Follower follower)
	{

		Instance = new PurpleMemory(hardwareMap, follower);
		return Instance;
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

	/**
	 * Gets persistent key/value storage.
	 */
	public Persist persist ()
	{

		return persist;
	}
}
