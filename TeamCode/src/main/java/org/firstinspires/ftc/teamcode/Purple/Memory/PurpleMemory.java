package org.firstinspires.ftc.teamcode.Purple.Memory;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Balls;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Position;

import java.util.HashMap;
import java.util.Map;

/**
 * Extended memory store
 */
public class PurpleMemory
{
	private final Map<String, Object> memory = new HashMap<>();
	private final Balls balls;
	private final Position position;

	/**
	 * Constructs PurpleMemory
	 */
	public PurpleMemory (HardwareMap hardwareMap)
	{

		balls = new Balls(hardwareMap);
		position = new Position(hardwareMap);
		memory.put("position", position);
	}

	/**
	 * Updates the robots memory of each component
	 */
	public void update ()
	{

		position.update();
		balls.update();
		memory.put("position", position);
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

	/**
	 * Direct access to the Position object
	 */
	public Position getPositionObject ()
	{

		return position;
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
