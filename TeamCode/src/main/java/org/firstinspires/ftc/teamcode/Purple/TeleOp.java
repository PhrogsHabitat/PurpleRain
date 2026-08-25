package org.firstinspires.ftc.teamcode.Purple;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.AutoRotateController;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
@Configurable
public class TeleOp extends PurpleOpMode
{
	public static double EXPLOSHER_DEFAULT_RPM = 2800.0;
	public static double EXPLOSHER_COAST_ALPHA = 0.08;
	private static final Pose DEFAULT_TELEOP_START_POSE = new Pose(72, 72, Math.toRadians(0.0));

	private static final String DRIVER_1_AUTO_ROTATE = "right_bumper";
	private static final String DRIVER_2_GATE_TOGGLE = "left_trigger";
	private static final String DRIVER_2_RPM_STEP_UP = "dpad_up";
	private static final String DRIVER_2_RPM_STEP_DOWN = "dpad_down";
	private static final String DRIVER_2_VACUUM_IN = "y";
	private static final String DRIVER_2_VACUUM_OUT = "x";
	private static final String DRIVER_2_VACUUM_SAFE_OUT = "b";

	public static Pose startingPose;

	private Controls driver1;
	private Controls driver2;
	private Follower follower;
	
	public Explosher explosher;

	private AutoRotateController autoRotateController;
	private Vaccum vaccum;
	private double desiredExplosherRPM = EXPLOSHER_DEFAULT_RPM;
	private double rememberedRegressedRPM = EXPLOSHER_DEFAULT_RPM;
	private boolean wasTagDetected = false;
	private Explosher.GateState gateState = Explosher.GateState.STOP;

	@Override
	public void create ()
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
		follower.setStartingPose(startingPose == null ? DEFAULT_TELEOP_START_POSE : startingPose);
		follower.update();
		follower.startTeleopDrive(true);

		PurpleMemory.initialize(hardwareMap, follower);
		LimeUtil.start(hardwareMap, 60);
		LimeUtil.setPipeline(0);

		autoRotateController = new AutoRotateController();
		autoRotateController.registerWithPanels();

		explosher = new Explosher(hardwareMap);
		explosher.setAimPoint(autoRotateController.getAimX(), autoRotateController.getAimY());
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

		explosher.setAimPoint(autoRotateController.getAimX(), autoRotateController.getAimY());
		explosher.update();
		vaccum.update();

		updateAprilTagFeedback();
		updateDrive();
		updateExplosher();
		updateGateState();
		updateVaccum();
		updateTelemetry();

		DebugUtil.update();
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
		double forward = -gamepad1.left_stick_y * Constants.DRIVE_POWER_SCALE;
		double strafe = -gamepad1.left_stick_x * Constants.DRIVE_POWER_SCALE;
		double turn = -gamepad1.right_stick_x * Constants.DRIVE_POWER_SCALE;

		if (driver1.isPressed(DRIVER_1_AUTO_ROTATE))
		{
			turn = autoRotateController.updateTurn(follower.getPose());
		}
		else
		{
			autoRotateController.reset();
		}

		follower.setTeleOpDrive(forward, strafe, turn, false);
	}

	private void updateExplosher ()
	{
		boolean manualRpmAdjust = Constants.DEBUG_MODE;
//				(driver2.justPressed(DRIVER_2_RPM_STEP_UP) || driver2.justPressed(DRIVER_2_RPM_STEP_DOWN));
		boolean useRegressionTarget = !manualRpmAdjust;

		explosher.setRegressionEnabled(useRegressionTarget);

		if (useRegressionTarget)
		{
			if (explosher.hasRegressionTarget())
			{
				rememberedRegressedRPM = explosher.getSmoothedTargetRPM();
			}

			if (explosher.hasRegressionTarget())
			{
				desiredExplosherRPM = rememberedRegressedRPM;
			}
			else
			{
				desiredExplosherRPM += EXPLOSHER_COAST_ALPHA * (EXPLOSHER_DEFAULT_RPM - desiredExplosherRPM);
			}
		}
		else
		{
			updateDebugRPM();
		}

		desiredExplosherRPM = Math.max(0.0, Math.min(desiredExplosherRPM, explosher.getMaxRPM()));
		explosher.setRPM(desiredExplosherRPM);
	}

	private void updateDebugRPM ()
	{
		if (!Constants.DEBUG_MODE)
		{
			return;
		}

		if (driver2.isPressed(DRIVER_2_RPM_STEP_UP))
		{
			desiredExplosherRPM += 100.0;
		}

		if (driver2.isPressed(DRIVER_2_RPM_STEP_DOWN))
		{
			desiredExplosherRPM -= 100.0;
		}
	}

	private void updateGateState ()
	{
		if (!driver2.justPressed(DRIVER_2_GATE_TOGGLE))
		{
			return;
		}

		explosher.toggleGateState();
		gateState = explosher.getGateState();
	}

	private void updateVaccum ()
	{
		if (driver2.isPressed(DRIVER_2_VACUUM_IN))
		{
			vaccum.setPower(Vaccum.DEFAULT_POW);
		}
		else if (driver2.isPressed(DRIVER_2_VACUUM_OUT))
		{
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		}
		else if (driver2.isPressed(DRIVER_2_VACUUM_SAFE_OUT))
		{
			vaccum.intakeMotor.setPower(Vaccum.DEFAULT_POW - 0.3);
			vaccum.midtakeMotor.setPower(-Vaccum.DEFAULT_POW);
		}
		else
		{
			vaccum.stop();
		}
	}

	private void updateTelemetry ()
	{
		Pose pose = follower.getPose();

		DebugUtil.logAdd("Distance to target: " + explosher.getDistanceToTarget());
		DebugUtil.logAdd("Auto Rotate Error: " + String.format("%.2f", autoRotateController.getLastHeadingErrorDeg()));
		DebugUtil.logAdd("Auto Rotate Turn: " + String.format("%.3f", autoRotateController.getLastTurnPower()));
		DebugUtil.logAdd("Auto Rotate Target: " + String.format("(%.1f, %.1f)", autoRotateController.getAimX(), autoRotateController.getAimY()));
		DebugUtil.logAdd("Auto Rotate Heading: " + String.format("%.2f", autoRotateController.getLastTargetHeadingDeg()));
		DebugUtil.logAdd("Target RPM: " + String.format("%.1f", explosher.getTargetRPM()));
		DebugUtil.logAdd("Current RPM: " + String.format("%.1f", explosher.getCurrentRPM()));
		DebugUtil.logAdd("Smoothed RPM: " + String.format("%.1f", explosher.getSmoothedTargetRPM()));
		DebugUtil.logAdd("Gate State: " + gateState);
		DebugUtil.logAdd("Gate Position: " + String.format("%.3f", explosher.getGatePosition()));
		DebugUtil.logAdd("Intake Power: " + String.format("%.2f", vaccum.getPower()));
		DebugUtil.logAdd("[FOLLOWER] POSE: " + pose);
		DebugUtil.logAdd("[LIME] TARGET: " + LimeUtil.hasValidTarget());
	}
}
