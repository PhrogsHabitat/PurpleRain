package org.firstinspires.ftc.teamcode.Purple.Memory;

import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
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

		DebugUtil.logAdd(memory.Balls().toString());

		// Update telemetry display
		DebugUtil.update();
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