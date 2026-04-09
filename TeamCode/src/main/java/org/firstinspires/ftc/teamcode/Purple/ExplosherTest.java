package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;

/**
 * Simple teleop for testing the shooter motor and blocker gate only.
 */
@TeleOp(name = "ExplosherTest", group = "Purple")
public class ExplosherTest extends OpMode
{
	private static final double JOYSTICK_DEADZONE = 0.08;

	private Explosher explosher;
	private boolean previousLeftTrigger;

	@Override
	public void init ()
	{
		explosher = new Explosher(hardwareMap);
		explosher.stop();
		telemetry.addData("Status", "Explosher test initialized");
	}

	@Override
	public void loop ()
	{
		double stickY = -gamepad1.left_stick_y;

		if (Math.abs(stickY) > JOYSTICK_DEADZONE)
		{
			explosher.setRPM(stickY * explosher.getMaxRPM());
		}
		else if (gamepad1.a)
		{
			explosher.setRPM(3000);
		}
		else if (gamepad1.x)
		{
			explosher.setRPM(4000);
		}
		else if (gamepad1.y)
		{
			explosher.setRPM(1000);
		}
		else if (gamepad1.b)
		{
			explosher.stop();
		}

		boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
		if (leftTriggerPressed && !previousLeftTrigger)
		{
			explosher.toggleGateState();
		}
		previousLeftTrigger = leftTriggerPressed;

		explosher.update();

		telemetry.addData("Current RPM", String.format("%.1f", explosher.getCurrentRPM()));
		telemetry.addData("Target RPM", String.format("%.1f", explosher.getTargetRPM()));
		telemetry.addData("Gate State", explosher.getGateState());
		telemetry.addData("Gate Pos", String.format("%.3f", explosher.getGatePosition()));
		telemetry.update();
	}

	@Override
	public void stop ()
	{
		if (explosher != null)
		{
			explosher.stop();
		}
	}
}
