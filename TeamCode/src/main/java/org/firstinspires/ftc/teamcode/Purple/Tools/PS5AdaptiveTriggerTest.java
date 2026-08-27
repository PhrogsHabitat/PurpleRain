package org.firstinspires.ftc.teamcode.Purple.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Purple.Controls;
import org.firstinspires.ftc.teamcode.Purple.PlayStationController;

@TeleOp(name = "PS5 Adaptive Trigger Test", group = "Tools")
public class PS5AdaptiveTriggerTest extends LinearOpMode
{
	private Controls driver1;
	private PlayStationController.TriggerMode currentMode = PlayStationController.TriggerMode.OFF;
	private int resistance = 128;
	private int startPos = 0;

	@Override
	public void runOpMode () throws InterruptedException
	{
		driver1 = new Controls(gamepad1, 0, hardwareMap.appContext);

		telemetry.addLine("PS5 Adaptive Trigger Diagnostic");
		telemetry.addLine("DPAD UP/DOWN: Change Resistance");
		telemetry.addLine("DPAD LEFT/RIGHT: Change Start Position");
		telemetry.addLine("A: Mode OFF, B: FEEDBACK, X: WEAPON, Y: VIBRATION");
		telemetry.update();

		waitForStart();

		while (opModeIsActive())
		{
			driver1.update();

			if (gamepad1.a) currentMode = PlayStationController.TriggerMode.OFF;
			if (gamepad1.b) currentMode = PlayStationController.TriggerMode.FEEDBACK;
			if (gamepad1.x) currentMode = PlayStationController.TriggerMode.WEAPON;
			if (gamepad1.y) currentMode = PlayStationController.TriggerMode.VIBRATION;

			if (gamepad1.dpad_up) resistance = Math.min(255, resistance + 5);
			if (gamepad1.dpad_down) resistance = Math.max(0, resistance - 5);
			if (gamepad1.dpad_right) startPos = Math.min(255, startPos + 5);
			if (gamepad1.dpad_left) startPos = Math.max(0, startPos - 5);

			// Apply settings
			if (currentMode == PlayStationController.TriggerMode.OFF)
			{
				driver1.setAdaptiveTrigger(false, currentMode);
				driver1.setAdaptiveTrigger(true, currentMode);
			}
			else if (currentMode == PlayStationController.TriggerMode.FEEDBACK)
			{
				driver1.setAdaptiveTrigger(false, currentMode, startPos, resistance);
				driver1.setAdaptiveTrigger(true, currentMode, startPos, resistance);
			}
			else if (currentMode == PlayStationController.TriggerMode.WEAPON)
			{
				// Weapon mode: start, end, force
				driver1.setAdaptiveTrigger(false, currentMode, startPos, startPos + 50, resistance);
				driver1.setAdaptiveTrigger(true, currentMode, startPos, startPos + 50, resistance);
			}
			else if (currentMode == PlayStationController.TriggerMode.VIBRATION)
			{
				// Vibration mode: start, amplitude, frequency
				driver1.setAdaptiveTrigger(false, currentMode, startPos, resistance, 10);
				driver1.setAdaptiveTrigger(true, currentMode, startPos, resistance, 10);
			}

			telemetry.addData("Mode", currentMode);
			telemetry.addData("Resistance", resistance);
			telemetry.addData("Start Pos", startPos);
			telemetry.addData("Trigger L", String.format("%.2f", gamepad1.left_trigger));
			telemetry.addData("Trigger R", String.format("%.2f", gamepad1.right_trigger));
			telemetry.update();
			
			sleep(50);
		}
	}
}
