package org.firstinspires.ftc.teamcode.Purple;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
public class TeleOp extends PurpleOpMode
{
	private static final double EXPLOSHER_DEFAULT_RPM = 2800.0;
	private static final double EXPLOSHER_COAST_ALPHA = 0.08;

	// Driver 2 explosher controls
	private static final String DRIVER_2_FINGER_TOGGLE = "left_trigger";
	private static final String DRIVER_2_HOOD_STEP_UP = "dpad_up";
	private static final String DRIVER_2_HOOD_STEP_DOWN = "dpad_down";

	// Driver 2 vaccum controls
	private static final String DRIVER_2_VACUUM_IN = "a";
	private static final String DRIVER_2_VACUUM_OUT = "b";
	private static final String DRIVER_2_VACUUM_SHOOT = "right_trigger";

	public static Pose startingPose;
	private Controls driver1;
	private Controls driver2;
	private Follower follower;
	private Explosher explosher;
	private Vaccum vaccum;

	private double desiredExplosherRPM = EXPLOSHER_DEFAULT_RPM;
	private double rememberedRegressedRPM = EXPLOSHER_DEFAULT_RPM;
	private boolean wasTagDetected = false;
	private Explosher.FingerState fingerState = Explosher.FingerState.STOP;

	@Override
	public void create ()
	{

		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
		follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
		follower.update();
		follower.startTeleopDrive(true);

		PurpleMemory.initialize(hardwareMap, follower);
		LimeUtil.start(hardwareMap, 60);
		LimeUtil.setPipeline(0);

		explosher = new Explosher(hardwareMap);
		explosher.setRegressionEnabled(false);

		vaccum = new Vaccum(hardwareMap);

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update ()
	{

		driver1.update();
		driver2.update();
		follower.update();
		LimeUtil.update();
		PurpleMemory.Instance.update();

		explosher.update();
		vaccum.update();

		updateAprilTagFeedback();
		updateDrive();
		updateExplosher();
		updateVaccum();
		teleInfo();
	}

	@Override
	public void destroy ()
	{

		explosher.stop();
		vaccum.stop();
	}

	private void updateAprilTagFeedback ()
	{

		boolean tagDetected = LimeUtil.hasValidTarget();
		if (tagDetected && !wasTagDetected)
		{
			driver1.vibrate(150);
			driver2.vibrate(Constants.VIBRATION_TAG_DETECTED);
		}

		wasTagDetected = tagDetected;
	}

	private void updateDrive ()
	{

		follower.setTeleOpDrive(
				-gamepad1.left_stick_y * Constants.DRIVE_POWER_SCALE,
				-gamepad1.left_stick_x * Constants.DRIVE_POWER_SCALE,
				-gamepad1.right_stick_x * Constants.DRIVE_POWER_SCALE,
				true
		);
	}

	private void updateExplosher ()
	{

		if (!Constants.DEBUG_MODE)
		{
			double leftStickY = driver2.getLeftStickY();
			boolean useRegressionTarget = leftStickY > Constants.JOYSTICK_DEADZONE;
			explosher.setRegressionEnabled(useRegressionTarget);

			if (useRegressionTarget)
			{
				if (LimeUtil.getTargetDistance() != 0)
				{
					rememberedRegressedRPM = explosher.getSmoothedTargetRPM();
				}

				desiredExplosherRPM = rememberedRegressedRPM;
			}
			else
			{
				desiredExplosherRPM += EXPLOSHER_COAST_ALPHA * (EXPLOSHER_DEFAULT_RPM - desiredExplosherRPM);
			}

			desiredExplosherRPM = Math.max(0.0, Math.min(desiredExplosherRPM, explosher.getMaxRPM()));
			explosher.setRPM(desiredExplosherRPM);
		}
		else
		{
			explosher.setRegressionEnabled(false);
			explosher.stop();
		}

		explosher.updateAim(PurpleMemory.Instance.curPos());
		updateFingerState();
		updateDebugHood();
	}

	private void updateFingerState ()
	{

		if (!driver2.justPressed(DRIVER_2_FINGER_TOGGLE))
		{
			return;
		}

		fingerState = fingerState == Explosher.FingerState.PASS ? Explosher.FingerState.STOP : Explosher.FingerState.PASS;
		explosher.setFingerState(fingerState);
	}

	private void updateDebugHood ()
	{

		if (!Constants.DEBUG_MODE)
		{
			return;
		}

		if (driver2.justPressed(DRIVER_2_HOOD_STEP_UP))
		{
			if (explosher.getFingerStateEnum() != Explosher.FingerState.DEBUG)
			{
				explosher.setFingerState(Explosher.FingerState.DEBUG);
			}

			explosher.adjustDebugFingerPosition(Constants.FINGER_DEBUG_INCREMENT);
			fingerState = explosher.getFingerStateEnum();
		}

		if (driver2.justPressed(DRIVER_2_HOOD_STEP_DOWN))
		{
			if (explosher.getFingerStateEnum() != Explosher.FingerState.DEBUG)
			{
				explosher.setFingerState(Explosher.FingerState.DEBUG);
			}

			explosher.adjustDebugFingerPosition(-Constants.FINGER_DEBUG_INCREMENT);
			fingerState = explosher.getFingerStateEnum();
		}
	}

	private void updateVaccum ()
	{

		if (driver2.justPressed(DRIVER_2_VACUUM_SHOOT))
		{
			vaccum.shoot();
		}

		if (driver2.isPressed(DRIVER_2_VACUUM_IN))
		{
			vaccum.setPower(Vaccum.DEFAULT_POW);
		}
		else if (driver2.isPressed(DRIVER_2_VACUUM_OUT))
		{
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		}
		else
		{
			vaccum.stop();
		}
	}

	private void teleInfo ()
	{

		DebugUtil.logAdd("============== [LIME]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Target X: " + LimeUtil.getTx());
		DebugUtil.logAdd("Target D: " + LimeUtil.getTargetDistance());
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("============== [EXPLOSHER]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Target RPM: " + explosher.getTargetRPM());
		DebugUtil.logAdd("Current RPM: " + explosher.getCurrentRPM());
		DebugUtil.logAdd("Smoothed Regress: " + explosher.getSmoothedTargetRPM());
		DebugUtil.logAdd("Auto Aim: ON");
		DebugUtil.logAdd("Exploring Pos: " + explosher.getExploringPos());
		DebugUtil.logAdd(String.format("Exploring PID: out=%.3f err=%.2f", explosher.getAimPow(), explosher.getAimErr()));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("============== [FINGER]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Finger State: " + fingerState);
		DebugUtil.logAdd("Finger Position: " + String.format("%.3f", explosher.getFingerPosition()));
		DebugUtil.logAdd(String.format("Vacuum Finger 1 Pos: %.3f", vaccum.getFingerPosition(0)));
		DebugUtil.logAdd(String.format("Vacuum Finger 2 Pos: %.3f", vaccum.getFingerPosition(1)));
		DebugUtil.logAdd(String.format("Vacuum Finger 3 Pos: %.3f", vaccum.getFingerPosition(2)));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("============== [MEMORY]");
		DebugUtil.logAdd("[MOTIF]: " + PurpleMemory.Instance.curMotif());
		DebugUtil.logAdd("[BALLS]: " + Arrays.toString(PurpleMemory.Instance.curBalls()));
		DebugUtil.logAdd(" ");

		Pose followerPose = follower.getPose();
		DebugUtil.logAdd("[POSITION] HEADING: " + Math.toDegrees(followerPose.getHeading()));
		DebugUtil.logAdd("[POSITION] X: " + PurpleMemory.Instance.curPos().getX());
		DebugUtil.logAdd("[POSITION] Y: " + PurpleMemory.Instance.curPos().getY());
		DebugUtil.logAdd(" ");

		DebugUtil.update();
	}
}
