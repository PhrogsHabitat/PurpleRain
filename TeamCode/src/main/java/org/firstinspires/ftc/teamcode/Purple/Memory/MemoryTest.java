package org.firstinspires.ftc.teamcode.Purple.Memory;

import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

/**
 * Test teleop for shared memory position/persist state using PurpleOpMode structure.
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
		DebugUtil.logAdd("Position memory smoke test");
		DebugUtil.update();
	}

	@Override
	public void update ()
	{
		memory.update();

		DebugUtil.logAdd(String.format(
				"Position: x=%.2f y=%.2f h=%.2f",
				memory.curPos().getX(),
				memory.curPos().getY(),
				memory.curPos().getHeading()
		));

		// Update telemetry display
		DebugUtil.update();
	}

	@Override
	public void destroy ()
	{

		DebugUtil.logAdd("=== Memory Test Complete ===");
		DebugUtil.logAdd("Stopping shared memory test");
		DebugUtil.update();
	}
}
