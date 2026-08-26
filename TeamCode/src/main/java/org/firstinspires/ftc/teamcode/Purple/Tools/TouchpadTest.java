package org.firstinspires.ftc.teamcode.Purple.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Purple.Controls;

@TeleOp(name = "Touchpad Test", group = "Tools")
public class TouchpadTest extends LinearOpMode
{
	private Controls driver1;
	private Controls driver2;

	@Override
	public void runOpMode () throws InterruptedException
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);
		telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
		telemetry.addLine("Press Start");
		telemetry.update();

		waitForStart();

		while (opModeIsActive())
		{
			driver1.update();
			driver2.update();

			logGamepad("Gamepad 1", gamepad1, driver1);
			telemetry.addLine();
			logGamepad("Gamepad 2", gamepad2, driver2);
			telemetry.update();
			sleep(20);
		}
	}

	private void logGamepad (String name, Gamepad gamepad, Controls controls)
	{
		telemetry.addLine(name);
		telemetry.addData("id/type/time", "%d / %s / %d", gamepad.id, gamepad.type(), gamepad.timestamp);
		telemetry.addData("click", gamepad.touchpad);
		telemetry.addData(
				"finger 1",
				"%s x=%5.2f y=%5.2f",
				gamepad.touchpad_finger_1,
				gamepad.touchpad_finger_1_x,
				gamepad.touchpad_finger_1_y
		);
		telemetry.addData(
				"finger 2",
				"%s x=%5.2f y=%5.2f",
				gamepad.touchpad_finger_2,
				gamepad.touchpad_finger_2_x,
				gamepad.touchpad_finger_2_y
		);
		telemetry.addData(
				"tracked",
				"%s x=%5.2f y=%5.2f",
				controls.isTouching(),
				controls.getTouchX(),
				controls.getTouchY()
		);
		telemetry.addData(
				"events",
				"down=%s up=%s tap=%s hold=%s swipe=%s",
				controls.justTouched(),
				controls.justReleasedTouch(),
				controls.justTapped(),
				controls.justHeld(),
				controls.getSwipe()
		);
	}
}
