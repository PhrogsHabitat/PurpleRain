package org.firstinspires.ftc.teamcode.Purple;

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
	private static final double RPM_SMOOTHING_ALPHA = 0.2;
	private static final double ROTATION_KP = 0.03; // Proportional gain for rotation
	private static final double ROTATION_DEADZONE = 0.5; // Degrees to consider "centered"
	private static final double MAX_ROTATION_POWER = 0.5; // Maximum rotation power
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
	private Explosher.FingerState fingerState = Explosher.FingerState.STOP;
	private double regressionSlope;
	private double regressionIntercept;
	private double smoothedTargetRPM = 0;
	// Rotation control variables
	private boolean autoRotateToTag = false;

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

	private void updateDrive ()
	{

		powerScale = driver1.isPressed("left_stick_button") ?
				Constants.DRIVE_POWER_BOOST : Constants.DRIVE_POWER_SCALE;

		double forward = driver1.getLeftStickY();
		double strafe = driver1.getLeftStickX();
		double turn = driver1.getRightStickX();

		// Auto-rotate to AprilTag when right bumper is held
		if (driver1.isPressed("right_bumper") && LimeUtil.hasValidTarget())
		{
			autoRotateToTag = true;
			double tx = LimeUtil.getTx(); // Horizontal offset in degrees

			// If we're outside the deadzone, calculate rotation power
			if (Math.abs(tx) > ROTATION_DEADZONE)
			{
				// Proportional control: more power when further from center
				double rotationPower = tx * ROTATION_KP;

				// Limit the rotation power
				if (rotationPower > MAX_ROTATION_POWER)
				{
					rotationPower = MAX_ROTATION_POWER;
				} else if (rotationPower < -MAX_ROTATION_POWER)
				{
					rotationPower = -MAX_ROTATION_POWER;
				}

				// Apply rotation power (negative because tx positive means tag is to the right)
				turn = -rotationPower;
			} else
			{
				// Tag is centered, don't rotate
				turn = 0;
				DebugUtil.logAdd("AprilTag Centered!");
			}

			if (Math.abs(tx) < ROTATION_DEADZONE)
			{
				driver1.vibrate(Constants.VIBRATION_ALIGNED);
			}
		} else
		{
			autoRotateToTag = false;
		}

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
		updatePlayer1Controls();
		updatePlayer2Controls();
		updateSubsystems();

		if (Constants.DEBUG_MODE)
		{
			updateDebug();
		}

		updateDrive();
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
			} else if (!manual)
			{
				explosher.stop();
			}
		} else if (leftStickY < -Constants.JOYSTICK_DEADZONE && driver2.isPressed("x"))
		{
			explosher.setRPM(-4000);
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		} else if (driver2.isPressed("b"))
		{
			explosher.setRPM(-4000);
			vaccum.swagReverse(-Vaccum.DEFAULT_POW);
		} else
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
				{65, 3000},
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

	private void updatePlayer1Controls ()
	{

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
	}

	private void updateDebug ()
	{

		DebugUtil.logAdd("Finger State: " + fingerState);
		DebugUtil.logAdd("Finger Position: " + String.format("%.3f", explosher.getFingerPosition()));
		DebugUtil.logAdd("Auto-Rotate Active: " + autoRotateToTag);
	}

	private void updateSubsystems ()
	{

		explosher.update();
		vaccum.update();
	}

	private void updateTelemetry ()
	{

		DebugUtil.logAdd("Target Distance: " + LimeUtil.getTargetDistance());
		DebugUtil.logAdd("Explosher Target RPM: " + String.format("%.1f", explosher.getTargetRPM()));
		DebugUtil.logAdd("Explosher Current RPM: " + String.format("%.1f", explosher.getCurrentRPM()));
		DebugUtil.logAdd("Finger State: " + fingerState);
		DebugUtil.logAdd("Finger Position: " + String.format("%.3f", explosher.getFingerPosition()));

		if (autoRotateToTag)
		{
			DebugUtil.logAdd("Auto-Rotate: ACTIVE (tx: " + String.format("%.1f", LimeUtil.getTx()) + "°)");
		}

		if (LimeUtil.hasValidTarget())
		{
			DebugUtil.logAdd("AprilTag - Dist: " + String.format("%.1f", LimeUtil.getTargetDistance()) +
					"in, Angle: " + String.format("%.1f", LimeUtil.getTx()) + "°");
		} else
		{
			DebugUtil.logAdd("AprilTag: No target");
		}

		DebugUtil.update();
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
	}
}