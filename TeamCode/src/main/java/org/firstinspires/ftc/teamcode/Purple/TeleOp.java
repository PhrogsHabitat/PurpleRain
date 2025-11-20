package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
public class TeleOp extends LinearOpMode
{
	private static final long TAG_TIMEOUT_MS = 500;
	private static final double RPM_SMOOTHING_ALPHA = 0.2; // Adjust smoothing factor (0.0 to 1.0)
	public double swagShitClose = Explosher.CLOSE_SWEET; // 900
	public double swagShitFar = Explosher.FAR_SWEET;   // 1600
	public double dist;
	private Controls driver1;
	private Controls driver2;
	private MotorConfig fl, fr, bl, br;
	private double powerScale = Constants.DRIVE_POWER_SCALE;
	private Explosher explosher;
	private Vaccum vaccum;
	// State tracking
	private boolean wasAligned = false;
	private boolean wasTagDetected = false;
	private Explosher.DistanceState stickyState = null;
	// Auto-align state
	private boolean autoAlignActive = false;
	private long lastTagSeenTime = 0;
	// Linear regression variables
	private double regressionSlope;
	private double regressionIntercept;
	// Add a smoothing factor for target RPM
	private double smoothedTargetRPM = 0;

	@Override
	public void runOpMode ()
	{

		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		initializeMotors();
		initializeExplosher();
		initializeVaccum();
		DebugUtil.setTelemetry(telemetry);

		// Calculate the regression line from calibration points
		calculateRegression();

		waitForStart();
		while (opModeIsActive())
		{
			driver1.update();
			driver2.update();
			update();
		}

		stopAll();
	}

	// 8 Hours so far on BLOOP

	// 7 (give or take) hours on the first video (COMPLETE)
	// 12 hours on CRASH Logo (COMPLETE)
	// 2 hours on Thumbnail (COMPLETE)
	// 2 hours (Give or take) on the "What is CRASH?" Video (COMPLETE)

	private void initializeMotors ()
	{

		fl = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.FRONT_LEFT, 2150.8, 312).build();
		fr = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.FRONT_RIGHT, 2150.8, 312).build();
		bl = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.BACK_LEFT, 2150.8, 312).build();
		br = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.BACK_RIGHT, 2150.8, 312).build();
	}

	private void initializeExplosher ()
	{
		// Lime camera + explosher init
		LimeUtil.start(hardwareMap, "SwagLime", 60);
		LimeUtil.setPipeline(0);
		explosher = new Explosher(hardwareMap, Constants.FINGER_SERVO_CONFIG);
	}

	private void initializeVaccum ()
	{

		vaccum = new Vaccum(hardwareMap);
	}

	private void calculateRegression ()
	{
		// The more points you add, the more accurate it becomes!
		double[][] calibrationPoints = {
				{80, 1200},
				{95, 1300},
				{105, 1300},
				{126, 1400},
		};

		int n = calibrationPoints.length;
		double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

		for (double[] point : calibrationPoints)
		{
			double distance = point[0];
			double rpm = point[1];
			sumX += distance;
			sumY += rpm;
			sumXY += distance * rpm;
			sumX2 += distance * distance;
		}

		// Calculate slope (m) and intercept (b) for: RPM = m * distance + b
		regressionSlope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
		regressionIntercept = (sumY - regressionSlope * sumX) / n;

		DebugUtil.logAdd("Regression Calculated!");
		DebugUtil.logAdd("RPM = " + String.format("%.3f", regressionSlope) + " * dist + " + String.format("%.3f", regressionIntercept));
	}

	private void update ()
	{

		LimeUtil.update();
		updateAprilTagFeedback();
		updatePlayer1Controls();
		updatePlayer2Controls();

		// Only run debug adjustments once per loop if enabled
		if (Constants.DEBUG_MODE)
		{
			updateDebug();
		}

		if (autoAlignActive)
		{
			autoAlignToTag();
		} else
		{
			updateDrive();
		}

		// UPDATED: Use linear regression to calculate RPM based on distance
		if (LimeUtil.getTargetDistance() != 0)
		{
			dist = LimeUtil.getTargetDistance();
			double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;

			// Smooth the target RPM using EMA
			smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);

			// Clamp the smoothed RPM to a reasonable range
			smoothedTargetRPM = Math.max(0, Math.min(smoothedTargetRPM, explosher.getMaxRPM()));

			explosher.setRPM(smoothedTargetRPM);
		}

		updateSubsystems();
		updateTelemetry();
	}

	// ===== DRIVE (unchanged directions & math) =====

	private void updateDrive ()
	{
		// Speed boost when left stick button is pressed
		powerScale = driver1.isPressed("left_stick_button") ?
				Constants.DRIVE_POWER_BOOST : Constants.DRIVE_POWER_SCALE;

		double forward = driver1.getLeftStickY();
		double strafe = driver1.getLeftStickX();
		double turn = driver1.getRightStickX();

		double[] powers = MotorUtil.normalizePowers(new double[]{
				(-forward - strafe - turn),
				(-forward + strafe - turn),
				(forward - strafe - turn),
				(forward + strafe - turn)
		});

		fl.setPower(powers[0] * powerScale);
		bl.setPower(powers[1] * powerScale);
		fr.setPower(powers[2] * powerScale);
		br.setPower(powers[3] * powerScale);
	}

	// ===== APRILTAG / TAG FEEDBACK =====

	private void updateAprilTagFeedback ()
	{

		boolean tagDetected = LimeUtil.hasValidTarget();

		if (tagDetected)
		{
			lastTagSeenTime = System.currentTimeMillis();
		}

		// Quick vibration when tag newly detected
		if (tagDetected && !wasTagDetected)
		{
			driver1.vibrate(Constants.VIBRATION_TAG_DETECTED);
			driver2.vibrate(Constants.VIBRATION_TAG_DETECTED);
		}

		wasTagDetected = tagDetected;
	}

	// ===== PLAYER 1 (driver) =====

	private void updatePlayer1Controls ()
	{
		// Auto-align with AprilTag when right bumper held and tag present
		autoAlignActive = driver1.isPressed("right_bumper") && LimeUtil.hasValidTarget();

		// Impact detection placeholder
		checkForImpact();
	}

	// ===== PLAYER 2 (operator) =====

	private void updatePlayer2Controls ()
	{
		// --- Explosher stick behavior (temporary manual control) ---
		double leftStickY = driver2.getLeftStickY();

		if (Math.abs(leftStickY) > Constants.JOYSTICK_DEADZONE)
		{
			// Stick being used: clear sticky state (temporary manual control)
			stickyState = null;

			if (driver2.isPressed("left_stick_button"))
			{
				// Stick pressed + forward => FAR sweet spot
				explosher.setRPM(swagShitFar);
				explosher.setMotorState(MotorConfig.MotorState.ON);
			} else if (leftStickY > 0.5)
			{
				// Stick forward => CLOSE sweet spot
				explosher.setRPM(swagShitClose);
				explosher.setMotorState(MotorConfig.MotorState.ON);
			}
			// NOTE: if stick used but not forwarded >0.5, we don't change RPM (keep previous)
		} else if (stickyState == null)
		{
			// Stick returned to center and no sticky preset => turn off
//			explosher.setMotorState(MotorConfig.MotorState.OFF);
		}

		// --- Explosher trigger-based sticky toggle (same behavior) ---
		if (driver2.justPressed("left_trigger"))
		{
			if (stickyState == null || stickyState == Explosher.DistanceState.FAR)
			{
				stickyState = Explosher.DistanceState.NEAR;
				explosher.setRPM(Explosher.CLOSE_SWEET);
			} else
			{
				stickyState = Explosher.DistanceState.FAR;
				explosher.setRPM(Explosher.FAR_SWEET);
			}
			explosher.setMotorState(MotorConfig.MotorState.ON);
		}

		// --- Vacuum control (operator) ---
		if (driver2.isPressed("y"))
		{
			vaccum.setState(MotorConfig.MotorState.ON);
		} else if (driver2.isPressed("x"))
		{
			vaccum.setState(MotorConfig.MotorState.ON);
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		} else
		{
			vaccum.setState(MotorConfig.MotorState.OFF);
		}

		// --- Debug dpad tweaks (keeps original behavior) ---
		// NOTE: updateDebug() is called once per loop if DEBUG_MODE is true;
		// we still allow driver2 to bump sweet spots here for quick tuning too.
		if (driver2.justPressed("dpad_up") && Constants.DEBUG_MODE)
		{
			swagShitClose += 100;
		}
		if (driver2.justPressed("dpad_down") && Constants.DEBUG_MODE)
		{
			swagShitClose -= 100;
		}
		if (driver2.justPressed("dpad_left") && Constants.DEBUG_MODE)
		{
			swagShitFar += 100;
		}
		if (driver2.justPressed("dpad_right") && Constants.DEBUG_MODE)
		{
			swagShitFar -= 100;
		}

		// --- Alignment vibration feedback for operator ---
		if (isFullyAligned() && !wasAligned)
		{
			driver2.vibrate(Constants.VIBRATION_ALIGNED);
		}
		wasAligned = isFullyAligned();
	}

	private void updateDebug ()
	{
		// (We already expose dpad adjustments inside updatePlayer2Controls for quick tuning.)
	}

	// ===== AUTO-ALIGN HELPERS =====

	private boolean hasRecentTarget ()
	{

		return LimeUtil.hasValidTarget() &&
				(System.currentTimeMillis() - lastTagSeenTime) < TAG_TIMEOUT_MS;
	}

	private void autoAlignToTag ()
	{

		if (!hasRecentTarget())
		{
			autoAlignActive = false;
			return;
		}

		double tx = LimeUtil.getTx(); // horizontal offset (deg)
		double distance = LimeUtil.getTargetDistance();

		// errors
		double angleError = -tx;
		double distanceError = Constants.DESIRED_TAG_DISTANCE - distance;

		// apply deadzone
		if (Math.abs(angleError) < Constants.ALIGN_ANGLE_DEADZONE) angleError = 0;
		if (Math.abs(distanceError) < Constants.ALIGN_DISTANCE_DEADZONE) distanceError = 0;

		double strafePower = clamp(angleError * Constants.ALIGN_ANGLE_KP,
				-Constants.MAX_ALIGN_POWER, Constants.MAX_ALIGN_POWER);

		double forwardPower = 0; // intentionally 0 for angle-only strafing
		double turnPower = clamp(-angleError * Constants.ALIGN_ANGLE_KP,
				-Constants.MAX_ALIGN_POWER, Constants.MAX_ALIGN_POWER);

		double[] powers = MotorUtil.normalizePowers(new double[]{
				(-forwardPower - strafePower - turnPower),
				(-forwardPower + strafePower - turnPower),
				(forwardPower - strafePower - turnPower),
				(forwardPower + strafePower - turnPower)
		});

		fl.setPower(powers[0] * powerScale);
		bl.setPower(powers[1] * powerScale);
		fr.setPower(powers[2] * powerScale);
		br.setPower(powers[3] * powerScale);

		// vibration feedback
		double alignmentError = Math.abs(angleError) + Math.abs(distanceError);
		if (alignmentError < 2.0)
		{
			driver1.vibrate(50); // continuous gentle
		} else if (alignmentError < 5.0)
		{
			if ((System.currentTimeMillis() % 500) < 250)
			{
				driver1.vibrate(25); // pulsed
			}
		}
	}

	private double clamp (double value, double min, double max)
	{

		return Math.max(min, Math.min(max, value));
	}

	private boolean isFullyAligned ()
	{

		if (!hasRecentTarget()) return false;

		double tx = LimeUtil.getTx();
		double distance = LimeUtil.getTargetDistance();
		double distanceError = Math.abs(distance - Constants.DESIRED_TAG_DISTANCE);

		return Math.abs(tx) < Constants.ALIGN_ANGLE_TOLERANCE &&
				distanceError < Constants.ALIGN_DISTANCE_TOLERANCE;
	}

	private void checkForImpact ()
	{
		// Placeholder
	}

	// ===== SUBSYSTEM UPDATES =====

	private void updateSubsystems ()
	{

		// Ensure the Explosher's update method is called
		explosher.update();
		vaccum.update();
	}

	// ===== TELEMETRY =====

	private void updateTelemetry ()
	{

		DebugUtil.logAdd("Target Distance: " + LimeUtil.getTargetDistance());
		DebugUtil.logAdd("Auto-Align: " + (autoAlignActive ? "ACTIVE" : "INACTIVE"));
		DebugUtil.logAdd("Sticky State: " + stickyState);

		// NEW: Show regression calculation info
		double calculatedRPM = (regressionSlope * dist) + regressionIntercept;
		DebugUtil.logAdd("Calculated RPM: " + String.format("%.1f", calculatedRPM) +
				" (from regression)");
		DebugUtil.logAdd("Smoothed Target RPM: " + String.format("%.1f", smoothedTargetRPM));
		DebugUtil.logAdd("Explosher Current RPM: " + String.format("%.1f", explosher.getCurrentRPM()));
		DebugUtil.logAdd("Explosher Target RPM: " + String.format("%.1f", explosher.getTargetRPM()));

		if (LimeUtil.hasValidTarget())
		{
			DebugUtil.logAdd("AprilTag - Dist: " + String.format("%.1f", LimeUtil.getTargetDistance()) +
					"in, Angle: " + String.format("%.1f", LimeUtil.getTx()) + "°");
			DebugUtil.logAdd("Aligned: " + (isFullyAligned() ? "YES" : "NO"));
		} else
		{
			DebugUtil.logAdd("AprilTag: No target");
		}

		DebugUtil.update();
	}

	private void stopAll ()
	{

		fl.stop();
		fr.stop();
		bl.stop();
		br.stop();
		explosher.setMotorState(MotorConfig.MotorState.OFF);
		vaccum.setState(MotorConfig.MotorState.OFF);
		autoAlignActive = false;
	}
}

