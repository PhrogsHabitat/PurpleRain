package org.firstinspires.ftc.teamcode.Purple;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

import java.util.ArrayList;
import java.util.Arrays;
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

	public static Pose startingPose;
	private Controls driver1;
	private Controls driver2;
	private Follower follower;
	private Explosher explosher;
	private Vaccum vaccum;

	private double desiredExplosherRPM = EXPLOSHER_DEFAULT_RPM;
	private double rememberedRegressedRPM = EXPLOSHER_DEFAULT_RPM;
	private double rememberedRegressedHood = Constants.FINGER_STOP_POSITION;
	private final List<double[]> debugRpmDataset = new ArrayList<>();
	private final List<double[]> debugHoodDataset = new ArrayList<>();
	private final RevColorSensorV3[] colorSensors = new RevColorSensorV3[6];
	private final String[] colorSensorLabels = new String[]{
			Names.COLOR1,
			Names.COLOR2,
			Names.COLOR3,
			Names.COLOR4,
			Names.COLOR5,
			Names.COLOR6
	};
	private boolean persistTuneDumpSection = false;
	private boolean wasTagDetected = false;
	private Explosher.FingerState fingerState = Explosher.FingerState.STOP;

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

		vaccum = new Vaccum(hardwareMap);
		initTempColorSensorLogging();

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
			updateDebugRPM();
			desiredExplosherRPM = Math.max(0.0, Math.min(desiredExplosherRPM, explosher.getMaxRPM()));
			explosher.setRPM(desiredExplosherRPM);
		}

		explosher.updateAim(PurpleMemory.Instance.curPos());
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

		desiredExplosherRPM = Math.max(0.0, Math.min(desiredExplosherRPM, explosher.getMaxRPM()));
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
			persistTuneDumpSection = true;
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

		DebugUtil.logAdd("============== [EXPLOSHER TUNE DUMP]");
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

	private void teleInfo ()
	{

		DebugUtil.logAdd("============== [LIME]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("POSE: " + LimeUtil.getResult().getBotpose());
		DebugUtil.logAdd("Target X: " + LimeUtil.getTx());
		DebugUtil.logAdd("Target D: " + LimeUtil.getTargetDistance());
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("============== [EXPLOSHER]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Target RPM: " + explosher.getTargetRPM());
		DebugUtil.logAdd("Current RPM: " + explosher.getCurrentRPM());
		DebugUtil.logAdd("Smoothed Regress: " + explosher.getSmoothedTargetRPM());
		DebugUtil.logAdd("Smoothed Hood Regress: " + explosher.getSmoothedTargetHoodPosition());
		Double odoDistInches = explosher.getDistanceToTarget();
		DebugUtil.logAdd("[ODOMETRY] Target Dist: " + (odoDistInches == null ? "N/A" : odoDistInches));
		if (Constants.DEBUG_MODE)
		{
			DebugUtil.logAdd(String.format(
					"[TUNE] Points | RPM: %d | HOOD: %d",
					debugRpmDataset.size(),
					debugHoodDataset.size()
			));
		}
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

		if (Constants.DEBUG_MODE && persistTuneDumpSection)
		{
			logDebugInterpolationDatasets();
			DebugUtil.logAdd(" ");
		}

		DebugUtil.update();
	}

	private void initTempColorSensorLogging ()
	{

		for (int i = 0; i < colorSensorLabels.length; i++)
		{
			colorSensors[i] = hardwareMap.get(RevColorSensorV3.class, colorSensorLabels[i]);
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
