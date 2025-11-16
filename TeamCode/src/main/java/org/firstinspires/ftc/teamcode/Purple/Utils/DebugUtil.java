package org.firstinspires.ftc.teamcode.Purple.Utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class DebugUtil
{
	private static final List<String> logs = new ArrayList<>();
	private static Telemetry telemetry;

	/**
	 * Gets the current telemetry object.
	 *
	 * @return The Telemetry instance.
	 */
	public static Telemetry getTelemetry ()
	{
		return telemetry;
	}

	/**
	 * Sets the telemetry object for debug output.
	 *
	 * @param t The Telemetry instance to use.
	 */
	public static void setTelemetry (Telemetry t)
	{
		telemetry = t;
	}

	/**
	 * Adds a message to the debug log.
	 *
	 * @param msg The message to add.
	 */
	public static void logAdd (String msg)
	{
		logs.add(msg);
	}

	/**
	 * Updates the telemetry with all log messages and clears the log.
	 */
	public static void update ()
	{
		if (telemetry != null)
		{
			for (String msg : logs)
			{
				telemetry.addLine(msg);
			}
			telemetry.update();
			logs.clear();
		}
	}
}
