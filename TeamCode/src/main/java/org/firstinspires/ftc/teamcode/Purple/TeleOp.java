package org.firstinspires.ftc.teamcode.Purple;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
public class TeleOp extends PurpleOpMode
{
	// Private variables after
	private static final long TAG_TIMEOUT_MS = 500;

	// Public variables first
	public double swagShitClose = 2500;
	public double swagShitFar = Explosher.FAR_SWEET;
	public double dist;
	public boolean manual = false;
	private double prevX;
	private Controls driver1;
	private Controls driver2;
	private MotorConfig fl, fr, bl, br;
	private double powerScale = Constants.DRIVE_POWER_SCALE;
	private Explosher explosher;
	private Vaccum vaccum;
	private PurpleMemory memory;
	private boolean wasAligned = false;
	private boolean wasTagDetected = false;
	private Explosher.FingerState fingerState = Explosher.FingerState.STOP;
	private boolean autoAlignActive = false;
	private long lastTagSeenTime = 0;
	private double smoothedTargetRPM = 0;

	// Core methods ALWAYS come first (excluding destroy)
	@Override
	public void create ()
	{

		memory = new PurpleMemory(hardwareMap);

		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		// Initialize motors (IF MANUAL TELEOP ONLY)
		fl = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.FRONT_LEFT, 2150.76, 312)
				.disableVelocityControl().build();
		fr = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.FRONT_RIGHT, 2150.76, 312)
				.disableVelocityControl().build();
		bl = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.BACK_LEFT, 2150.76, 312)
				.disableVelocityControl().build();
		br = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.BACK_RIGHT, 2150.76, 312)
				.disableVelocityControl().build();

		// Initialize LimeLight
		LimeUtil.start(hardwareMap, "SwagLime", 60);
		LimeUtil.setPipeline(0);

		// Initialize Explosher
		explosher = new Explosher(hardwareMap, Constants.FINGER_SERVO_CONFIG);

		// Initialize Vaccum
		vaccum = new Vaccum(hardwareMap);

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update ()
	{

		// We always gotta update the controls first!
		driver1.update();
		driver2.update();
		memory.update();

		LimeUtil.update();
		explosher.update();
		vaccum.update();

		autoAlignActive = driver1.isPressed("right_bumper") && LimeUtil.hasValidTarget();

		// Update each control-based module
		updateAprilTagFeedback();
		updateExplosher();
		updateVaccum();
		teleInfo();

		updateDrive();

		// Here we can update the debug controls, ONLY if DEBUG_MODE is true
		if (Constants.DEBUG_MODE)
		{
			updateDebug();
		}
	}

	private void updateDrive ()
	{

		powerScale = driver1.isPressed("left_stick_button") ? Constants.DRIVE_POWER_BOOST : Constants.DRIVE_POWER_SCALE;

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

	private void updateExplosher ()
	{

		double leftStickY = driver2.getLeftStickY();

		if (leftStickY > Constants.JOYSTICK_DEADZONE)
		{
			if (LimeUtil.getTargetDistance() != 0)
			{
				explosher.shouldRegress = true;

				if (!manual)
				{
					explosher.setRPM(explosher.smoothedTargetRPM);
				}
			} else if (!manual)
			{
				explosher.shouldRegress = false;
				explosher.stop();
			}
		} else if (leftStickY < -Constants.JOYSTICK_DEADZONE && driver2.isPressed("x"))
		{
			explosher.setRPM(-4000);
		} else
		{
			explosher.stop();
		}

		if (autoAlignActive)
		{
			double tx = LimeUtil.getTx();

			if (tx != 0.0 && LimeUtil.hasValidTarget())
			{
				prevX = tx;
				double power = clamp(tx / 2, -1, 1);
				explosher.setRingPower(power);
			} else
			{
				double power = clamp(prevX / 2, -1, 1);
			}

		} else
		{
			// Manual control
			double rightStickX = driver2.getRightStickX();
			if (Math.abs(rightStickX) > Constants.JOYSTICK_DEADZONE)
			{
				explosher.setRingPower(rightStickX);
			} else
			{
				explosher.setRingPower(0);
			}
		}

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

		if (driver2.isPressed("a"))
		{
			explosher.resetRingPosition();
		}

		if (driver2.isPressed("b"))
		{
			explosher.setRingPosition(20, 0.5);
		}
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

	private void updateVaccum ()
	{

		if (driver2.isPressed("y"))
		{
			vaccum.setPower(Vaccum.DEFAULT_POW);
		} else if (driver2.isPressed("x"))
		{
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		} else
		{
			vaccum.stop();
		}

		if (isFullyAligned() && !wasAligned)
		{
			driver2.vibrate(Constants.VIBRATION_ALIGNED);
		}
		wasAligned = isFullyAligned();
	}

	private void updateDebug ()
	{

		if (driver2.justPressed("dpad_up"))
		{
			manual = true;
			swagShitClose += 100;
		}

		if (driver2.justPressed("dpad_down"))
		{
			manual = true;
			swagShitClose -= 100;
		}

		if (driver2.justPressed("dpad_left"))
		{
			manual = false;
			swagShitFar += 100;
			explosher.stop();
		}

		if (driver2.justPressed("dpad_right"))
		{
			swagShitFar -= 100;
		}
	}

	private void teleInfo ()
	{

		DebugUtil.logAdd("======== [LIME]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Target X: " + LimeUtil.getTx());
		DebugUtil.logAdd("Target D: " + LimeUtil.getTargetDistance());
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("======= [EXPLOSHER]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Target RPM: " + String.format("%.1f", explosher.getTargetRPM()));
		DebugUtil.logAdd("Current RPM: " + String.format("%.1f", explosher.getCurrentRPM()));
		DebugUtil.logAdd("Auto-Align: " + (autoAlignActive ? "ACTIVE" : "INACTIVE"));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("======= [FINGER]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Finger State: " + fingerState);
		DebugUtil.logAdd("Finger Position: " + String.format("%.3f", explosher.getFingerPosition()));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("======= [MEMORY]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Position" + memory.get("position"));
		DebugUtil.logAdd(" ");

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
