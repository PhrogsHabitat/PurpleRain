package org.firstinspires.ftc.teamcode.Purple;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
public class TeleOp extends PurpleOpMode
{
	private static final double EXPLOSHER_DEFAULT_RPM = 2800.0;
	private static final double EXPLOSHER_COAST_ALPHA = 0.08;
	private static final double EXPLOSHER_DEBUG_RPM_INCREMENT = 100.0;
	private static final Pose DEFAULT_TELEOP_START_POSE = new Pose(72, 72, Math.toRadians(0.0));

	// Driver 2 explosher controls
	private static final String DRIVER_2_FINGER_TOGGLE = "left_trigger";
	private static final String DRIVER_2_RPM_STEP_UP = "dpad_up";
	private static final String DRIVER_2_RPM_STEP_DOWN = "dpad_down";
	private static final String DRIVER_2_HOOD_STEP_UP = "dpad_right";
	private static final String DRIVER_2_HOOD_STEP_DOWN = "dpad_left";
	private static final String DRIVER_2_CAPTURE_HOOD_POINT = "select";
	private static final String DRIVER_2_CAPTURE_RPM_POINT = "start";
	private static final String DRIVER_2_CAPTURE_PRINT_MODIFIER = "left_trigger";

	// Driver 2 vaccum controls
	private static final String DRIVER_2_VACUUM_IN = "a";
	private static final String DRIVER_2_VACUUM_OUT = "b";
	private static final String DRIVER_2_VACUUM_SHOOT = "right_trigger";
	private static final long POSE_CORRECTION_INTERVAL_MS = 1000; // once per second
	public static Pose startingPose;
	private final List<double[]> debugRpmDataset = new ArrayList<>();
	private final List<double[]> debugHoodDataset = new ArrayList<>();
	private Controls driver1;
	private Controls driver2;
	private Follower follower;
	private Explosher explosher;
	private Vaccum vaccum;
	private double desiredExplosherRPM = EXPLOSHER_DEFAULT_RPM;
	private double rememberedRegressedRPM = EXPLOSHER_DEFAULT_RPM;
	private double rememberedRegressedHood = Constants.FINGER_STOP_POSITION;
	private boolean wasTagDetected = false;
	private Explosher.FingerState fingerState = Explosher.FingerState.STOP;
	// For periodic pose correction
	private long lastCorrectionTime = 0;

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

		explosher = new Explosher(hardwareMap);
		explosher.setRegressionEnabled(false);
		if (Constants.DEBUG_MODE)
		{
			desiredExplosherRPM = 0.0;
			rememberedRegressedRPM = 0.0;
		}

		// Set the aim point to the desired basket (change based on alliance)
		explosher.setAimPoint(128, 130);

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

		// Periodic pose correction using LimeLight AprilTag detection
		long now = System.currentTimeMillis();
//		if (now - lastCorrectionTime > POSE_CORRECTION_INTERVAL_MS)
//		{
//			Pose limelightPose = LimeUtil.getRobotPose();
//			if (limelightPose != null)
//			{
//				follower.setPose(limelightPose);
//				lastCorrectionTime = now;
//				DebugUtil.logAdd("Pose corrected using LimeLight");
//			}
//		}

		DebugUtil.logAdd("Distance from tag: " + explosher.getDistanceToTarget());
		DebugUtil.logAdd("EXPLO DEGREE: " + explosher.getExploringDeg());
		DebugUtil.logAdd("TARGET DEGREE: " + explosher.getExploringTargetDeg());
		DebugUtil.logAdd("EXPLO ERROR: " + explosher.getAimErr());
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("[LIME] POSE: " + LimeUtil.getRobotPose());
		DebugUtil.logAdd("[FOLLOWER] POSE: " + follower.getPose());

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

		follower.setTeleOpDrive(
				-gamepad1.left_stick_y * Constants.DRIVE_POWER_SCALE,
				-gamepad1.left_stick_x * Constants.DRIVE_POWER_SCALE,
				-gamepad1.right_stick_x * Constants.DRIVE_POWER_SCALE,
				false
		);
	}

	private void updateExplosher ()
	{

		boolean manualRpmDebugAdjust = Constants.DEBUG_MODE && (driver2.isPressed(DRIVER_2_RPM_STEP_UP) || driver2.isPressed(DRIVER_2_RPM_STEP_DOWN));

		boolean useRegressionTarget = true;
		if (manualRpmDebugAdjust)
		{
			useRegressionTarget = false;
		}

		explosher.setRegressionEnabled(useRegressionTarget);

		if (useRegressionTarget)
		{
			if (explosher.hasRegressionTarget())
			{
				rememberedRegressedRPM = explosher.getSmoothedTargetRPM();
				rememberedRegressedHood = explosher.getSmoothedTargetHoodPosition();
			}

			desiredExplosherRPM = rememberedRegressedRPM;
			if (explosher.getFingerStateEnum() != Explosher.FingerState.DEBUG)
			{
				explosher.setFingerPosition(rememberedRegressedHood);
			}
		}
		else if (manualRpmDebugAdjust)
		{
			updateDebugRPM();
		}
		else
		{
			desiredExplosherRPM += EXPLOSHER_COAST_ALPHA * (EXPLOSHER_DEFAULT_RPM - desiredExplosherRPM);
		}

		desiredExplosherRPM = Math.max(0.0, Math.min(desiredExplosherRPM, explosher.getMaxRPM()));
		explosher.setRPM(desiredExplosherRPM);
		explosher.updateAim(follower.getPose());
		updateDebugHood();
		updateDebugInterpolationCapture();
		updateFingerState();
	}

	private void updateDebugRPM ()
	{

		if (!Constants.DEBUG_MODE)
		{
			return;
		}

		if (driver2.isPressed(DRIVER_2_RPM_STEP_UP))
		{
			desiredExplosherRPM += EXPLOSHER_DEBUG_RPM_INCREMENT;
		}

		if (driver2.isPressed(DRIVER_2_RPM_STEP_DOWN))
		{
			desiredExplosherRPM -= EXPLOSHER_DEBUG_RPM_INCREMENT;
		}
	}

	private void updateDebugHood ()
	{

		if (!Constants.DEBUG_MODE)
		{
			return;
		}

		if (driver2.isPressed(DRIVER_2_HOOD_STEP_UP))
		{
			if (explosher.getFingerStateEnum() != Explosher.FingerState.DEBUG)
			{
				explosher.setFingerState(Explosher.FingerState.DEBUG);
			}

			explosher.adjustDebugFingerPosition(Constants.FINGER_DEBUG_INCREMENT);
			fingerState = explosher.getFingerStateEnum();
		}

		if (driver2.isPressed(DRIVER_2_HOOD_STEP_DOWN))
		{
			if (explosher.getFingerStateEnum() != Explosher.FingerState.DEBUG)
			{
				explosher.setFingerState(Explosher.FingerState.DEBUG);
			}

			explosher.adjustDebugFingerPosition(-Constants.FINGER_DEBUG_INCREMENT);
			fingerState = explosher.getFingerStateEnum();
		}
	}

	private void updateDebugInterpolationCapture ()
	{

		if (!Constants.DEBUG_MODE)
		{
			return;
		}

		boolean printComboPressed = driver2.isPressed(DRIVER_2_CAPTURE_PRINT_MODIFIER) &&
				driver2.justPressed(DRIVER_2_CAPTURE_RPM_POINT);
		if (printComboPressed)
		{
			logDebugInterpolationDatasets();
			return;
		}

		if (driver2.justPressed(DRIVER_2_CAPTURE_HOOD_POINT))
		{
			captureDebugHoodPoint();
		}

		if (driver2.justPressed(DRIVER_2_CAPTURE_RPM_POINT))
		{
			captureDebugRpmPoint();
		}
	}

	private void captureDebugHoodPoint ()
	{

		Double distanceInches = explosher.getDistanceToTarget();
		if (distanceInches == null)
		{
			DebugUtil.logAdd("[TUNE] HOOD point skipped: odometry distance unavailable.");
			return;
		}

		double hoodPosition = explosher.getFingerPosition();
		debugHoodDataset.add(new double[]{distanceInches, hoodPosition});
		DebugUtil.logAdd(String.format(
				"[TUNE] Saved HOOD point: {%.2f, %.3f} (count=%d)",
				distanceInches,
				hoodPosition,
				debugHoodDataset.size()
		));
	}

	private void captureDebugRpmPoint ()
	{

		Double distanceInches = explosher.getDistanceToTarget();
		if (distanceInches == null)
		{
			DebugUtil.logAdd("[TUNE] RPM point skipped: odometry distance unavailable.");
			return;
		}

		double rpmValue = explosher.getTargetRPM();
		debugRpmDataset.add(new double[]{distanceInches, rpmValue});
		DebugUtil.logAdd(String.format(
				"[TUNE] Saved RPM point: {%.2f, %.0f} (count=%d)",
				distanceInches,
				rpmValue,
				debugRpmDataset.size()
		));
	}

	private void logDebugInterpolationDatasets ()
	{

		logCalibrationDataset("RPM_CALIBRATION_POINTS", debugRpmDataset, true);
		logCalibrationDataset("HOOD_CALIBRATION_POINTS", debugHoodDataset, false);
	}

	private void logCalibrationDataset (String datasetName, List<double[]> dataset, boolean rpmDataset)
	{

		if (dataset.isEmpty())
		{
			DebugUtil.logAdd(datasetName + " is empty.");
			return;
		}

		List<double[]> sortedDataset = new ArrayList<>(dataset);
		sortedDataset.sort((a, b) -> Double.compare(a[0], b[0]));

		DebugUtil.logAdd("private static final double[][] " + datasetName + " = {");
		for (double[] point : sortedDataset)
		{
			String valueString = rpmDataset ?
					String.format("%.0f", point[1]) :
					String.format("%.3f", point[1]);
			DebugUtil.logAdd(String.format("\t\t{%.2f, %s},", point[0], valueString));
		}
		DebugUtil.logAdd("};");
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

	private void updateFingerState ()
	{

		if (Constants.DEBUG_MODE && driver2.isPressed(DRIVER_2_CAPTURE_RPM_POINT))
		{
			return;
		}

		if (!driver2.justPressed(DRIVER_2_FINGER_TOGGLE))
		{
			return;
		}

		fingerState = fingerState == Explosher.FingerState.PASS ? Explosher.FingerState.STOP : Explosher.FingerState.PASS;
		explosher.setFingerState(fingerState);
	}
}