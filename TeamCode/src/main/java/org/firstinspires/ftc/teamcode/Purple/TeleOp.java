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
	private static final double RPM_SMOOTHING_ALPHA = 0.2;
	public double swagShitClose = Explosher.CLOSE_SWEET;
	public double swagShitFar = Explosher.FAR_SWEET;
	public double dist;
	public boolean manual = false;
	private Controls driver1;
	private Controls driver2;
	private MotorConfig fl, fr, bl, br;
	private double powerScale = Constants.DRIVE_POWER_SCALE;
	private Explosher explosher;
	private Vaccum vaccum;
	private boolean wasAligned = false;
	private boolean wasTagDetected = false;
	private Explosher.DistanceState stickyState = null;
	private boolean autoAlignActive = false;
	private long lastTagSeenTime = 0;
	private double regressionSlope;
	private double regressionIntercept;
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

	private void initializeMotors ()
	{

		fl = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.FRONT_LEFT, 2150.76, 312)
				.disableVelocityControl().build();
		fr = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.FRONT_RIGHT, 2150.76, 312)
				.disableVelocityControl().build();
		bl = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.BACK_LEFT, 2150.76, 312)
				.disableVelocityControl().build();
		br = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.BACK_RIGHT, 2150.76, 312)
				.disableVelocityControl().build();
	}

	private void initializeExplosher ()
	{

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

		double[][] calibrationPoints = {
				{59, 3000},
				{65, 2800},
				{77, 3100},
				{80, 3200},
				{94, 3100}

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

		double leftStickY = driver2.getLeftStickY();

		if (leftStickY > Constants.JOYSTICK_DEADZONE)
		{
			if (LimeUtil.getTargetDistance() != 0)
			{
				dist = LimeUtil.getTargetDistance();
				double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;
				smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
				smoothedTargetRPM = Math.max(0, Math.min(smoothedTargetRPM, explosher.getMaxRPM()));
				explosher.setRPM(smoothedTargetRPM);
			}
			else
			{
				if (!manual)
				{
					explosher.stop();
				}
			}
		}

		else if (leftStickY < -Constants.JOYSTICK_DEADZONE && driver2.isPressed("x"))
		{
			explosher.setRPM(-4000);
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		}

		else if (driver2.isPressed(("b")))
		{
			explosher.setRPM(-4000);
			vaccum.swagReverse(-Vaccum.DEFAULT_POW);
		}

		else
		{
			explosher.stop();
		}

		if (driver2.isPressed("x"))
		{
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		}

		updateSubsystems();
		updateTelemetry();
	}

	/**
	 * Updates the drivetrain motors based on gamepad input
	 */
	private void updateDrive ()
	{

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

	private void updateAprilTagFeedback ()
	{

		boolean tagDetected = LimeUtil.hasValidTarget();

		if (tagDetected)
		{
			lastTagSeenTime = System.currentTimeMillis();
		}

		if (tagDetected && !wasTagDetected)
		{
			driver1.vibrate(150);
			driver2.vibrate(Constants.VIBRATION_TAG_DETECTED);
		}

		wasTagDetected = tagDetected;
	}

	private void updatePlayer1Controls ()
	{

		autoAlignActive = driver1.isPressed("right_bumper") && LimeUtil.hasValidTarget();
		checkForImpact();
	}

	private void updatePlayer2Controls ()
	{

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
		}

		if (driver2.isPressed("y"))
		{
			vaccum.setPower(Vaccum.DEFAULT_POW);
		}

		else
		{
			vaccum.stop();
		}

		if (driver2.justPressed("dpad_up") && Constants.DEBUG_MODE)
		{
			manual = true;
			swagShitClose += 100;
			explosher.setRPM(swagShitClose);
		}
		if (driver2.justPressed("dpad_down") && Constants.DEBUG_MODE)
		{
			manual = true;
			swagShitClose -= 100;
			explosher.setRPM(swagShitClose);
		}
		if (driver2.justPressed("dpad_left") && Constants.DEBUG_MODE)
		{
			manual = false;
			swagShitFar += 100;
			explosher.stop();
		}
		if (driver2.justPressed("dpad_right") && Constants.DEBUG_MODE)
		{
			swagShitFar -= 100;
		}

		if (isFullyAligned() && !wasAligned)
		{
			driver2.vibrate(Constants.VIBRATION_ALIGNED);
		}
		wasAligned = isFullyAligned();
	}

	private void updateDebug ()
	{
		// Debug adjustments if needed
	}

	private boolean hasRecentTarget ()
	{

		return LimeUtil.hasValidTarget() &&
				(System.currentTimeMillis() - lastTagSeenTime) < TAG_TIMEOUT_MS;
	}

	/**
	 * Automatically aligns the robot to the detected AprilTag
	 */
	private void autoAlignToTag() {

		if (!hasRecentTarget()) {
			autoAlignActive = false;
			return;
		}

		double tx = LimeUtil.getTx();                 // horizontal angle
		double distance = LimeUtil.getTargetDistance();
		double distanceError = Constants.DESIRED_TAG_DISTANCE - distance;

		// --- PD/P tuning ---
		double turnPower =
				clamp(tx * Constants.ALIGN_ANGLE_KP, -Constants.MAX_ALIGN_POWER, Constants.MAX_ALIGN_POWER);

		double forwardPower =
				clamp(distanceError * Constants.ALIGN_DISTANCE_KP, -Constants.MAX_ALIGN_POWER, Constants.MAX_ALIGN_POWER);

		// Dead zones
		if (Math.abs(tx) < Constants.ALIGN_ANGLE_DEADZONE) turnPower = 0;
		if (Math.abs(distanceError) < Constants.ALIGN_DISTANCE_DEADZONE) forwardPower = 0;

		// No strafe unless tag is severely off axis
		double strafePower = 0;
		if (Math.abs(tx) > 10) {
			strafePower =
					clamp(tx * Constants.ALIGN_STRAFE_KP, -Constants.MAX_ALIGN_POWER, Constants.MAX_ALIGN_POWER);
		}

		// --- Mecanum drive calculation ---
		double flPower = forwardPower - strafePower - turnPower;
		double frPower = forwardPower + strafePower - turnPower;
		double blPower = forwardPower - strafePower - turnPower;
		double brPower = forwardPower + strafePower - turnPower;

		double[] norm = MotorUtil.normalizePowers(new double[]{ flPower, frPower, blPower, brPower });

		fl.setPower(norm[0] * powerScale);
		fr.setPower(norm[1] * powerScale);
		bl.setPower(norm[2] * powerScale);
		br.setPower(norm[3] * powerScale);

		// --- Haptic feedback ---
		if (isFullyAligned()) {
			driver1.vibrate(80);
		}
	}


	/**
	 * Clamps a value between minimum and maximum bounds
	 *
	 * @param value The value to clamp
	 * @param min   Minimum allowed value
	 * @param max   Maximum allowed value
	 * @return The clamped value
	 */
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
		// Placeholder for impact detection
	}

	/**
	 * Updates all subsystem components
	 */
	private void updateSubsystems ()
	{

		explosher.update();
		vaccum.update();
	}

	/**
	 * Updates telemetry display with current robot state
	 */
	private void updateTelemetry ()
	{

		DebugUtil.logAdd("Target Distance: " + LimeUtil.getTargetDistance());
		DebugUtil.logAdd("Explosher Target RPM: " + String.format("%.1f", explosher.getTargetRPM()));
		DebugUtil.logAdd("Explosher Current RPM: " + String.format("%.1f", explosher.getCurrentRPM()));

		DebugUtil.logAdd("Auto-Align: " + (autoAlignActive ? "ACTIVE" : "INACTIVE"));
		DebugUtil.logAdd("Sticky State: " + stickyState);

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

	/**
	 * Stops all motors and subsystems
	 */
	private void stopAll ()
	{

		fl.stop();
		fr.stop();
		bl.stop();
		br.stop();
		explosher.stop();
		vaccum.stop();
		autoAlignActive = false;
	}
}