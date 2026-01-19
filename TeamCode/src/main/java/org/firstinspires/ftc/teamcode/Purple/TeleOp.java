package org.firstinspires.ftc.teamcode.Purple;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
public class TeleOp extends PurpleOpMode
{
	// Private variables after
	private static final long TAG_TIMEOUT_MS = 500;
	private static final double RPM_SMOOTHING_ALPHA = 0.2;
	private static final double THRESHOLD = 3;

	// Public variables first
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
	private Explosher.FingerState fingerState = Explosher.FingerState.STOP;
	private boolean autoAlignActive = false;
	private long lastTagSeenTime = 0;
	private double regressionSlope;
	private double regressionIntercept;
	private double smoothedTargetRPM = 0;

	// Core methods ALWAYS come first (excluding destroy)
	@Override
	public void create ()
	{

		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		initializeMotors();
		initializeExplosher();
		initializeVaccum();
		calculateRegression();

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update ()
	{

		driver1.update();
		driver2.update();

		updateAllSystems();
	}

	private void updateDrive (String dir)
	{

		powerScale = driver1.isPressed("left_stick_button") ?
				Constants.DRIVE_POWER_BOOST : Constants.DRIVE_POWER_SCALE;

		if (dir == "L")
		{
			double forward = 0;
			double strafe = 0;
			double turn = -0.15;

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
		else if (dir == "R")
		{
			double forward = 0;
			double strafe = 0;
			double turn = 0.15;

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
		else
		{
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
	}

	/**
	 * Automatically aligns the robot to the detected AprilTag
	 */

	// Private non-utility methods without documentation at bottom
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

	private void updateAllSystems ()
	{

		LimeUtil.update();
		updateAprilTagFeedback();
		updatePlayer1Controls();
		updatePlayer2Controls();
		updateSubsystems();

		if (Constants.DEBUG_MODE)
		{
			updateDebug();
		}

		if (autoAlignActive)
		{
			if (Math.abs(LimeUtil.getTx()) > THRESHOLD)
			{
				if(LimeUtil.getTx() < 0)
				{
					updateDrive("L");
				}
				else if(LimeUtil.getTx() > 0)
				{
					updateDrive("R");
				}
				else
				{
					updateDrive("def");
				}
			}
			else
			{
				driver1.vibrate(150);
				updateDrive("def");
			}
		}
		else
		{
			updateDrive("def");
		}

		updateExplosherControl();
		updateTelemetry();
	}

	private void updateExplosherControl ()
	{

		double leftStickY = driver2.getLeftStickY();

		if (leftStickY > Constants.JOYSTICK_DEADZONE)
		{
			if (LimeUtil.getTargetDistance() != 0)
			{
				dist = LimeUtil.getTargetDistance();
				double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;
				smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
				smoothedTargetRPM = Math.max(0, Math.min(smoothedTargetRPM, explosher.getMaxRPM()));

				if (!manual)
				{
					explosher.setRPM(smoothedTargetRPM);
				}
			}
			else if (!manual)
			{
				explosher.stop();
			}
		}
		else if (leftStickY < -Constants.JOYSTICK_DEADZONE && driver2.isPressed("x"))
		{
			explosher.setRPM(-4000);
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		}
		else if (driver2.isPressed("b"))
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
	}

	private void calculateRegression ()
	{

		double[][] calibrationPoints = {
				{59, 2900},
				{65, 2850},
				{77, 3000},
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
			explosher.cycleFingerState();
			fingerState = explosher.getFingerStateEnum();
		}

		if (fingerState == Explosher.FingerState.DEBUG)
		{
			if (driver2.justPressed("left_bumper"))
			{
				explosher.adjustDebugFingerPosition(-Constants.FINGER_DEBUG_INCREMENT);
			}
			if (driver2.justPressed("right_bumper"))
			{
				explosher.adjustDebugFingerPosition(Constants.FINGER_DEBUG_INCREMENT);
			}
		}

		if (driver2.isPressed("y"))
		{
			vaccum.setPower(Vaccum.DEFAULT_POW);
		} else
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

		DebugUtil.logAdd("Finger State: " + fingerState);
		DebugUtil.logAdd("Finger Position: " + String.format("%.3f", explosher.getFingerPosition()));
	}

	private void updateSubsystems ()
	{

		explosher.update();
		vaccum.update();
	}

	private void updateTelemetry ()
	{
		DebugUtil.logAdd("TX: " + LimeUtil.getTx());
		DebugUtil.logAdd("Target Distance: " + LimeUtil.getTargetDistance());
		DebugUtil.logAdd("Explosher Target RPM: " + String.format("%.1f", explosher.getTargetRPM()));
		DebugUtil.logAdd("Explosher Current RPM: " + String.format("%.1f", explosher.getCurrentRPM()));
		DebugUtil.logAdd("Auto-Align: " + (autoAlignActive ? "ACTIVE" : "INACTIVE"));
		DebugUtil.logAdd("Finger State: " + fingerState);
		DebugUtil.logAdd("Finger Position: " + String.format("%.3f", explosher.getFingerPosition()));

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

	private boolean hasRecentTarget ()
	{

		return LimeUtil.hasValidTarget() &&
				(System.currentTimeMillis() - lastTagSeenTime) < TAG_TIMEOUT_MS;
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
		// Placeholder for impact detection
	}

	@Override
	public void destroy ()
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

//{57, 2500},
//{80, 2700},
//{95, 2900},
//{110, 2900},
//{130, 3100}