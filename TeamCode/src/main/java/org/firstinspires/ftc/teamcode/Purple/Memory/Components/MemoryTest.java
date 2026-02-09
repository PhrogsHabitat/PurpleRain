package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

/**
 * Test teleop for ball memory system using PurpleOpMode structure
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Memory Test", group = "Purple")
public class MemoryTest extends PurpleOpMode
{
	private PurpleMemory memory;

	@Override
	public void create ()
	{
		// Initialize memory system with hardware map
		memory = new PurpleMemory(hardwareMap);

		// Setup DebugUtil with telemetry
		DebugUtil.setTelemetry(telemetry);

		DebugUtil.logAdd("=== Memory Test Initialized ===");
		DebugUtil.logAdd("Place balls in slots to test detection");
		DebugUtil.logAdd("Slot 1, 2, 3 correspond to color sensors");
		DebugUtil.update();
	}

	@Override
	public void update ()
	{
		// Update ball detection
		memory.onUpdate();

		// Get current ball status array
		int[] balls = memory.Balls();

		// Display status using DebugUtil
		DebugUtil.logAdd("=== Ball Status ===");

		for (int i = 0; i < balls.length; i++)
		{
			String status;
			switch (balls[i])
			{
				case 0:
					status = "Empty";
					break;
				case 1:
					status = "Purple";
					break;
				case 2:
					status = "Green";
					break;
				default:
					status = "Unknown";
			}
			DebugUtil.logAdd("Slot " + (i + 1) + ": " + status);
		}

		// Calculate summary statistics
		DebugUtil.logAdd("");
		DebugUtil.logAdd("=== Summary ===");

		int purpleCount = 0;
		int greenCount = 0;
		int emptyCount = 0;

		for (int ball : balls)
		{
			if (ball == 0) emptyCount++;
			else if (ball == 1) purpleCount++;
			else if (ball == 2) greenCount++;
		}

		DebugUtil.logAdd("Purple Balls: " + purpleCount);
		DebugUtil.logAdd("Green Balls: " + greenCount);
		DebugUtil.logAdd("Empty Slots: " + emptyCount);
		DebugUtil.logAdd("Total Detected: " + (purpleCount + greenCount));

		// Add optional raw sensor data for debugging
		if (memory.getBallsObject() != null)
		{
			DebugUtil.logAdd("");
			DebugUtil.logAdd("=== Quick Checks ===");
			DebugUtil.logAdd("Has Purple: " + (purpleCount > 0 ? "YES" : "NO"));
			DebugUtil.logAdd("Has Green: " + (greenCount > 0 ? "YES" : "NO"));
			DebugUtil.logAdd("Is Empty: " + (emptyCount == 3 ? "YES" : "NO"));
		}

		// Update telemetry display
		DebugUtil.update();

		// Small delay to prevent overwhelming telemetry
		try
		{
			Thread.sleep(50);
		} catch (InterruptedException e)
		{
			Thread.currentThread().interrupt();
		}
	}

	@Override
	public void destroy ()
	{

		DebugUtil.logAdd("=== Memory Test Complete ===");
		DebugUtil.logAdd("Stopping ball detection system");
		DebugUtil.update();

		// Clean up resources if needed
		// (The Balls class currently doesn't have a cleanup method)
	}
}