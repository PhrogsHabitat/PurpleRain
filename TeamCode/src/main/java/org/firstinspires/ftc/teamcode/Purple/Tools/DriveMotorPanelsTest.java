package org.firstinspires.ftc.teamcode.Purple.Tools;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Panels;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Names;

@TeleOp(name = "Drive Motor Panels Test", group = "Purple")
public class DriveMotorPanelsTest extends OpMode
{
	private static final double TEST_POWER = 0.25;

	private final String[] motorLabels = {"FL", "FR", "BL", "BR"};
	private final MotorConfig[] motors = new MotorConfig[4];

	private TelemetryManager panelsTelemetry;
	private int selectedMotorIndex = 0;
	private boolean previousDpadLeft;
	private boolean previousDpadRight;

	@Override
	public void init ()
	{
		panelsTelemetry = Panels.telemetry();

		motors[0] = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.FRONT_LEFT, 2150.76, 312)
				.disableVelocityControl().build();
		motors[1] = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.FRONT_RIGHT, 2150.76, 312)
				.disableVelocityControl().build();
		motors[2] = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.BACK_LEFT, 2150.76, 312)
				.disableVelocityControl().build();
		motors[3] = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.BACK_RIGHT, 2150.76, 312)
				.disableVelocityControl().build();
	}

	@Override
	public void loop ()
	{
		handleSelection();
		stopAll();

		String command = "Idle";
		if (gamepad1.a)
		{
			motors[selectedMotorIndex].setPower(TEST_POWER);
			command = "Selected forward";
		}
		else if (gamepad1.b)
		{
			motors[selectedMotorIndex].setPower(-TEST_POWER);
			command = "Selected reverse";
		}
		else if (gamepad1.x)
		{
			motors[0].setPower(TEST_POWER);
			motors[2].setPower(TEST_POWER);
			command = "Left side forward";
		}
		else if (gamepad1.y)
		{
			motors[1].setPower(TEST_POWER);
			motors[3].setPower(TEST_POWER);
			command = "Right side forward";
		}

		for (MotorConfig motor : motors)
		{
			motor.update();
		}

		panelsTelemetry.debug("Drive motor orientation test");
		panelsTelemetry.debug("DPad L/R: select motor");
		panelsTelemetry.debug("A/B: run selected forward/reverse");
		panelsTelemetry.debug("X: run left side, Y: run right side");
		panelsTelemetry.debug("Selected motor: " + motorLabels[selectedMotorIndex]);
		panelsTelemetry.debug("Command: " + command);
		panelsTelemetry.update(telemetry);
	}

	@Override
	public void stop ()
	{
		stopAll();
	}

	private void handleSelection ()
	{
		boolean dpadLeftPressed = gamepad1.dpad_left && !previousDpadLeft;
		boolean dpadRightPressed = gamepad1.dpad_right && !previousDpadRight;

		if (dpadLeftPressed)
		{
			selectedMotorIndex = (selectedMotorIndex + motors.length - 1) % motors.length;
		}

		if (dpadRightPressed)
		{
			selectedMotorIndex = (selectedMotorIndex + 1) % motors.length;
		}

		previousDpadLeft = gamepad1.dpad_left;
		previousDpadRight = gamepad1.dpad_right;
	}

	private void stopAll ()
	{
		for (MotorConfig motor : motors)
		{
			if (motor != null)
			{
				motor.setPower(0.0);
			}
		}
	}
}
