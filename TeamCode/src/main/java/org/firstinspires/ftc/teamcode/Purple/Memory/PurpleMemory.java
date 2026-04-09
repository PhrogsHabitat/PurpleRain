package org.firstinspires.ftc.teamcode.Purple.Memory;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Persist;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Position;

/**
 * Shared robot state container for frequently read, changing values.
 */
public class PurpleMemory
{
	public static PurpleMemory Instance;

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
	}

	/**
	 * Gets the current odometry position.
	 */
	public Position curPos ()
	{

		return position;
	}

	/**
	 * Gets the current odometry pose in Pedro coordinates.
	 */
	public Pose curPose ()
	{

		return new Pose(
				position.getX(),
				position.getY(),
				Math.toRadians(position.getHeading())
		);
	}

	/**
	 * Gets persistent key/value storage.
	 */
	public Persist persist ()
	{

		return persist;
	}
}
