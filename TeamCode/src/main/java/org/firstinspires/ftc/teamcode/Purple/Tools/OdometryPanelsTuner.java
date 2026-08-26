package org.firstinspires.ftc.teamcode.Purple.Tools;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Panels;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Persist;

@TeleOp(name = "Odometry Panels Tuner", group = "Purple")
public class OdometryPanelsTuner extends OpMode
{
	private static final String KEY_FORWARD_POD_X = "purple.odometry.forwardPodX";
	private static final String KEY_STRAFE_POD_Y = "purple.odometry.strafePodY";
	private static final String KEY_FORWARD_DIR = "purple.odometry.forwardDir";
	private static final String KEY_STRAFE_DIR = "purple.odometry.strafeDir";
	private static final double POD_OFFSET_STEP = 0.25;
	private static final Pose START_POSE = new Pose(72, 72, 0.0);

	private TelemetryManager panelsTelemetry;
	private Persist persist;
	private Follower follower;
	private double forwardPodX;
	private double strafePodY;
	private GoBildaPinpointDriver.EncoderDirection forwardDirection;
	private GoBildaPinpointDriver.EncoderDirection strafeDirection;
	private boolean editingForwardPodX = true;

	private boolean previousDpadUp;
	private boolean previousDpadDown;
	private boolean previousDpadLeft;
	private boolean previousX;
	private boolean previousY;
	private boolean previousA;

	@Override
	public void init ()
	{
		panelsTelemetry = Panels.telemetry();
		Panels.initPedroField();

		persist = new Persist(hardwareMap);
		follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
		follower.setStartingPose(START_POSE);

		forwardPodX = readPersistedDouble(
				KEY_FORWARD_POD_X,
				org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants.strafePodX);
		strafePodY = readPersistedDouble(
				KEY_STRAFE_POD_Y,
				org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants.forwardPodY);
		forwardDirection = readPersistedDirection(
				KEY_FORWARD_DIR,
				org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants.forwardEncoderDirection);
		strafeDirection = readPersistedDirection(
				KEY_STRAFE_DIR,
				org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants.strafeEncoderDirection);

		applyPinpointSettings();
		follower.update();
	}

	@Override
	public void start ()
	{
		follower.startTeleopDrive(true);
	}

	@Override
	public void loop ()
	{
		handleTuningInputs();

		follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
		follower.update();

		panelsTelemetry.debug("Odometry direction / offset tuner");
		panelsTelemetry.debug("DPad Left: switch edited offset");
		panelsTelemetry.debug("DPad Up/Down: change selected offset by " + POD_OFFSET_STEP + " in");
		panelsTelemetry.debug("X: toggle forward encoder direction");
		panelsTelemetry.debug("Y: toggle strafe encoder direction");
		panelsTelemetry.debug("A: reset pose + IMU");
		panelsTelemetry.debug("Editing: " + (editingForwardPodX ? KEY_FORWARD_POD_X : KEY_STRAFE_POD_Y));
		panelsTelemetry.debug(KEY_FORWARD_POD_X + ": " + forwardPodX);
		panelsTelemetry.debug(KEY_STRAFE_POD_Y + ": " + strafePodY);
		panelsTelemetry.debug(KEY_FORWARD_DIR + ": " + forwardDirection);
		panelsTelemetry.debug(KEY_STRAFE_DIR + ": " + strafeDirection);
		panelsTelemetry.debug("Pose X: " + follower.getPose().getX());
		panelsTelemetry.debug("Pose Y: " + follower.getPose().getY());
		panelsTelemetry.debug("Pose H: " + Math.toDegrees(follower.getPose().getHeading()));
		panelsTelemetry.update(telemetry);

		Panels.drawRobot(follower.getPose());
		Panels.send();
	}

	private void handleTuningInputs ()
	{
		boolean dpadUpPressed = gamepad1.dpad_up && !previousDpadUp;
		boolean dpadDownPressed = gamepad1.dpad_down && !previousDpadDown;
		boolean dpadLeftPressed = gamepad1.dpad_left && !previousDpadLeft;
		boolean xPressed = gamepad1.x && !previousX;
		boolean yPressed = gamepad1.y && !previousY;
		boolean aPressed = gamepad1.a && !previousA;

		if (dpadLeftPressed)
		{
			editingForwardPodX = !editingForwardPodX;
		}

		if (dpadUpPressed || dpadDownPressed)
		{
			double delta = dpadUpPressed ? POD_OFFSET_STEP : -POD_OFFSET_STEP;
			if (editingForwardPodX)
			{
				forwardPodX += delta;
			}
			else
			{
				strafePodY += delta;
			}

			saveSettings();
			applyPinpointSettings();
		}

		if (xPressed)
		{
			forwardDirection = toggleDirection(forwardDirection);
			saveSettings();
			applyPinpointSettings();
		}

		if (yPressed)
		{
			strafeDirection = toggleDirection(strafeDirection);
			saveSettings();
			applyPinpointSettings();
		}

		if (aPressed)
		{
			resetPinpoint();
		}

		previousDpadUp = gamepad1.dpad_up;
		previousDpadDown = gamepad1.dpad_down;
		previousDpadLeft = gamepad1.dpad_left;
		previousX = gamepad1.x;
		previousY = gamepad1.y;
		previousA = gamepad1.a;
	}

	private void applyPinpointSettings ()
	{
		if (!(follower.getPoseTracker().getLocalizer() instanceof PinpointLocalizer))
		{
			return;
		}

		PinpointLocalizer localizer = (PinpointLocalizer) follower.getPoseTracker().getLocalizer();
		localizer.getPinpoint().setOffsets(forwardPodX, strafePodY, DistanceUnit.INCH);
		localizer.getPinpoint().setEncoderDirections(forwardDirection, strafeDirection);
	}

	private void resetPinpoint ()
	{
		if (!(follower.getPoseTracker().getLocalizer() instanceof PinpointLocalizer))
		{
			return;
		}

		PinpointLocalizer localizer = (PinpointLocalizer) follower.getPoseTracker().getLocalizer();
		localizer.getPinpoint().resetPosAndIMU();
		follower.setPose(START_POSE);
	}

	private void saveSettings ()
	{
		persist.set(KEY_FORWARD_POD_X, String.valueOf(forwardPodX));
		persist.set(KEY_STRAFE_POD_Y, String.valueOf(strafePodY));
		persist.set(KEY_FORWARD_DIR, forwardDirection.name());
		persist.set(KEY_STRAFE_DIR, strafeDirection.name());
	}

	private double readPersistedDouble (String key, double fallback)
	{
		String value = persist.get(key);
		if (value == null)
		{
			return fallback;
		}

		try
		{
			return Double.parseDouble(value);
		}
		catch (NumberFormatException ignored)
		{
			return fallback;
		}
	}

	private GoBildaPinpointDriver.EncoderDirection readPersistedDirection (
			String key,
			GoBildaPinpointDriver.EncoderDirection fallback)
	{
		String value = persist.get(key);
		if (value == null)
		{
			return fallback;
		}

		try
		{
			return GoBildaPinpointDriver.EncoderDirection.valueOf(value);
		}
		catch (IllegalArgumentException ignored)
		{
			return fallback;
		}
	}

	private GoBildaPinpointDriver.EncoderDirection toggleDirection (GoBildaPinpointDriver.EncoderDirection direction)
	{
		return direction == GoBildaPinpointDriver.EncoderDirection.FORWARD ?
				GoBildaPinpointDriver.EncoderDirection.REVERSED :
				GoBildaPinpointDriver.EncoderDirection.FORWARD;
	}
}
