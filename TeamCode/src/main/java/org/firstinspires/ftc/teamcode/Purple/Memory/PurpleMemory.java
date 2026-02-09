package org.firstinspires.ftc.teamcode.Purple.Memory;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Balls;

import java.util.HashMap;
import java.util.Map;

/**
 * Extended memory store with ball tracking
 */
public class PurpleMemory
{
	private final Map<String, Object> memory = new HashMap<>();
	private final Balls balls;

	/**
	 * Constructs PurpleMemory with ball tracking
	 */
	public PurpleMemory (HardwareMap hardwareMap)
	{

		balls = new Balls(hardwareMap);
	}

	/**
	 * Updates ball detection - call in main loop
	 */
	public void onUpdate ()
	{

		balls.update();
	}

	/**
	 * Gets the current ball status array
	 * 0 = none, 1 = purple, 2 = green
	 */
	public int[] Balls ()
	{

		return balls.getBallStatus();
	}

	/**
	 * Direct access to the Balls object
	 */
	public Balls getBallsObject ()
	{

		return balls;
	}

	public void put (String key, Object value)
	{

		memory.put(key, value);
	}

	public Object get (String key)
	{

		return memory.get(key);
	}

	public boolean containsKey (String key)
	{

		return memory.containsKey(key);
	}

	public void clear ()
	{

		memory.clear();
	}
}