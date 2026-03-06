package org.firstinspires.ftc.teamcode.Purple;

import com.pedropathing.follower.Follower;
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
	private static final long TAG_TIMEOUT_MS = 500;
	private Pose startPose = new Pose(72, 72, 3 * Math.PI / 4);
	private static final double RPM_SMOOTHING_ALPHA = 0.2;
	private static final double THRESHOLD = 3;

	public double swagShitClose = Explosher.CLOSE_SWEET;
	public double swagShitFar = Explosher.FAR_SWEET;
	public double dist;
	public boolean manual = true;

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
	private double debugManualRPM = 0;

	public Follower follower;


	@Override
	public void create()
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
		follower.setPose(startPose);

		fl = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.FRONT_LEFT, 2150.76, 312)
				.disableVelocityControl().build();
		fr = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.FRONT_RIGHT, 2150.76, 312)
				.disableVelocityControl().build();
		bl = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.BACK_LEFT, 2150.76, 312)
				.disableVelocityControl().build();
		br = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.BACK_RIGHT, 2150.76, 312)
				.disableVelocityControl().build();

		LimeUtil.start(hardwareMap, "SwagLime", 60);
		LimeUtil.setPipeline(0);

		explosher = new Explosher(hardwareMap, Constants.FINGER_SERVO_CONFIG);
		vaccum = new Vaccum(hardwareMap);

		calculateRegression();

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update()
	{
		driver1.update();
		driver2.update();
		follower.update();

		updateAllSystems();
	}

	private void updateAllSystems()
	{
		LimeUtil.update(follower.getHeading());
		explosher.update();
		vaccum.update();

		updateAprilTagFeedback();
		updateDebugRpmControl();
		updateExplosher();
		updateVaccum();
		teleInfo();

		autoAlignActive = driver1.isPressed("right_bumper") && LimeUtil.hasValidTarget();

		if (autoAlignActive && Math.abs(LimeUtil.getTx()) > THRESHOLD)
		{
			updateDrive(LimeUtil.getTx() < 0 ? "L" : "R");
		}
		else
		{
			if (autoAlignActive)
				driver1.vibrate(150);

			updateDrive("def");
		}
	}

	private void updateDrive(String dir)
	{
		powerScale = driver1.isPressed("left_stick_button") ?
				Constants.DRIVE_POWER_BOOST : Constants.DRIVE_POWER_SCALE;

		if (dir.equals("L") || dir.equals("R"))
		{
			double turn = dir.equals("L") ? -0.15 : 0.15;

			double[] powers = MotorUtil.normalizePowers(new double[]{
					-turn, -turn, -turn, -turn
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

	private void updateExplosher()
	{
		double leftStickY = driver2.getLeftStickY();

		if (leftStickY > Constants.JOYSTICK_DEADZONE)
		{
			if (Constants.DEBUG_MODE && manual)
			{
				explosher.setRPM(-debugManualRPM);
			}
			else if (LimeUtil.hasValidTarget())
			{
				dist = LimeUtil.getTargetDistance();

				double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;

				smoothedTargetRPM += RPM_SMOOTHING_ALPHA *
						(rawTargetRPM - smoothedTargetRPM);

				smoothedTargetRPM = Math.max(0,
						Math.min(smoothedTargetRPM, explosher.getMaxRPM()));

				if (!manual)
					explosher.setRPM(-smoothedTargetRPM);
			}
			else if (!manual)
			{
				explosher.setRPM(-swagShitClose); // fallback if tag briefly drops
			}
		}
		else if (leftStickY < -Constants.JOYSTICK_DEADZONE)
		{
			explosher.setRPM(4000);
		}
		else
		{
			explosher.stop();
		}
		// Finger Control
		if (driver2.justPressed(("right_bumper")))
		{
			explosher.cycleFingerState();
		}
	}

	private void updateDebugRpmControl()
	{
		if (!Constants.DEBUG_MODE)
		{
			manual = false;
			return;
		}

		if (driver2.justPressed("dpad_up"))
		{
			if (!manual)
			{
				debugManualRPM = explosher.getTargetRPM();
			}
			manual = true;
			debugManualRPM = Math.min(explosher.getMaxRPM(), debugManualRPM + 100);
		}

		if (driver2.justPressed("dpad_down"))
		{
			if (!manual)
			{
				debugManualRPM = explosher.getTargetRPM();
			}
			manual = true;
			debugManualRPM = Math.max(0, debugManualRPM - 100);
		}

		if (driver2.justPressed("dpad_left"))
		{
			manual = false;
		}
	}

	private void updateVaccum()
	{
		if (driver2.isPressed("x"))
		{
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		}
		else if (driver2.isPressed("b"))
		{
			vaccum.swagReverse(-Vaccum.DEFAULT_POW);
		}
		else if (driver2.isPressed("y"))
		{
			vaccum.setPower(Vaccum.DEFAULT_POW);
		}
		else
		{
			vaccum.stop();
		}

		if (isFullyAligned() && !wasAligned)
			driver2.vibrate(Constants.VIBRATION_ALIGNED);

		wasAligned = isFullyAligned();
	}

	private void calculateRegression()
	{
		double[][] calibrationPoints = {
				{123, -4900},
				{80, -4100},
				{54, -3900},
				{47, -3700},
				{157, -5000}
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

		regressionSlope = (n * sumXY - sumX * sumY) /
				(n * sumX2 - sumX * sumX);

		regressionIntercept = (sumY - regressionSlope * sumX) / n;
	}

	private void updateAprilTagFeedback()
	{
		boolean tagDetected = LimeUtil.hasValidTarget();

		if (tagDetected)
			lastTagSeenTime = System.currentTimeMillis();

		if (tagDetected && !wasTagDetected)
		{
			driver1.vibrate(150);
			driver2.vibrate(Constants.VIBRATION_TAG_DETECTED);
		}

		wasTagDetected = tagDetected;
	}

	private boolean hasRecentTarget()
	{
		return LimeUtil.hasValidTarget() &&
				(System.currentTimeMillis() - lastTagSeenTime) < TAG_TIMEOUT_MS;
	}

	private boolean isFullyAligned()
	{
		if (!hasRecentTarget()) return false;

		double tx = LimeUtil.getTx();
		double distance = LimeUtil.getTargetDistance();
		double distanceError = Math.abs(distance - Constants.DESIRED_TAG_DISTANCE);

		return Math.abs(tx) < Constants.ALIGN_ANGLE_TOLERANCE &&
				distanceError < Constants.ALIGN_DISTANCE_TOLERANCE;
	}

	private void teleInfo()
	{
		DebugUtil.logAdd("[MT1] BotPose - " + LimeUtil.getResult().getBotpose());
		DebugUtil.logAdd("[MT2] BotPose - " + LimeUtil.getResult().getBotpose_MT2());

		DebugUtil.logAdd("TX: " + LimeUtil.getTx());
		DebugUtil.logAdd("Target Distance: " + LimeUtil.getTargetDistance());
		DebugUtil.logAdd("Explosher Target RPM: " + explosher.getTargetRPM());
		DebugUtil.logAdd("Explosher Current RPM: " + explosher.getCurrentRPM());
		DebugUtil.logAdd("Manual RPM Mode: " + (manual ? "ON" : "OFF"));
		DebugUtil.logAdd("Manual RPM Setpoint: " + debugManualRPM);
		DebugUtil.logAdd("Auto-Align: " + (autoAlignActive ? "ACTIVE" : "INACTIVE"));

		DebugUtil.update();
	}

	@Override
	public void destroy()
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
