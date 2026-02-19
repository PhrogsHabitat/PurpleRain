package org.firstinspires.ftc.teamcode.Purple;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
public class TeleOp extends PurpleOpMode
{
	// Private variables after
	private static final long TAG_TIMEOUT_MS = 500;
	private static final double DRIVE_ALIGN_TX_THRESHOLD = 3.0;
	private static final double SERVO_DEBUG_STEP = 0.01;
	// Public variables first
	public static Pose startingPose;
	public double exploringIncrement = 2500;
	public double swagShitFar = Explosher.FAR_SWEET;
	public double dist;
	public boolean manual = false;
	private int inc = 0;
	private Controls driver1;
	private Controls driver2;
	private Follower follower;
	private MotorConfig fl, fr, bl, br;
	private double powerScale = Constants.DRIVE_POWER_SCALE;
	private Explosher explosher;
	private Vaccum vaccum;
	private ColorSensor colorSensor2;
	private PurpleMemory memory;
	private boolean wasAligned = false;
	private boolean wasTagDetected = false;
	private Explosher.FingerState fingerState = Explosher.FingerState.STOP;
	private boolean autoAlignActive = false;
	private long lastTagSeenTime = 0;
	private int selectedDebugServo = 0;

	// Core methods ALWAYS come first (excluding destroy)
	@Override
	public void create ()
	{

		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);
		follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
		follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
		follower.update();
		follower.startTeleopDrive(true);

		memory = new PurpleMemory(hardwareMap);

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
		LimeUtil.start(hardwareMap, 60);
		LimeUtil.setPipeline(0);

		// Initialize Explosher
		explosher = new Explosher(hardwareMap);

		// Initialize Vaccum
		vaccum = new Vaccum(hardwareMap);

		// Debug color sensor (slot 2)
		colorSensor2 = hardwareMap.get(ColorSensor.class, Names.COLOR2);

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update ()
	{

		// We always gotta update the controls first!
		driver1.update();
		driver2.update();
		memory.update();
		follower.update();

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

		if (autoAlignActive)
		{
			if (Math.abs(LimeUtil.getTx()) > DRIVE_ALIGN_TX_THRESHOLD)
			{
				if (LimeUtil.getTx() < 0)
				{
					updateDrive("L");
				} else if (LimeUtil.getTx() > 0)
				{
					updateDrive("R");
				} else
				{
					updateDrive("def");
				}
			} else
			{
				driver1.vibrate(150);
				updateDrive("def");
			}
		} else
		{
			updateDrive("def");
		}
	}

	private void updateDrive (String dir)
	{

		powerScale = driver1.isPressed("left_stick_button") ? Constants.DRIVE_POWER_BOOST : Constants.DRIVE_POWER_SCALE;

		switch (dir)
		{
			case "L":
				follower.turnDegrees(10, true);
				break;
			case "R":
				follower.turnDegrees(10, false);
				break;
			default:
				follower.setTeleOpDrive(
						-gamepad1.left_stick_y * powerScale,
						-gamepad1.left_stick_x * powerScale,
						-gamepad1.right_stick_x * powerScale,
						true
				);
				break;
		}
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
					explosher.setRPM(-explosher.smoothedTargetRPM);
				}
			} else if (!manual)
			{
				explosher.shouldRegress = false;
				explosher.stop();
			}
		} else if (leftStickY < -Constants.JOYSTICK_DEADZONE && driver2.isPressed("x"))
		{
			explosher.setRPM(4000);
		} else
		{
			explosher.stop();
		}

		double rightStickX = driver2.getRightStickX();
		explosher.shouldAim = Math.abs(rightStickX) <= Constants.JOYSTICK_DEADZONE;
		explosher.updateAim(follower.getPose(), rightStickX);

		if (driver2.justPressed("left_trigger"))
		{
			explosher.cycleFingerState();
			fingerState = explosher.getFingerStateEnum();
		}

		if (driver2.isPressed("a"))
		{
			explosher.resetExploringPos();
		}

		if (driver2.justPressed("left_trigger"))
		{
			inc -= 20;
		}

		if (driver2.isPressed("b"))
		{
			explosher.setExploringPos(inc, 0.5);
			inc += 5;
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

		if (driver2.justPressed("left_stick_button"))
		{
			vaccum.flickFinger(0);
		}

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

		if (driver2.isPressed("dpad_up"))
		{
			vaccum.adjustFingerPosition(0, SERVO_DEBUG_STEP);
			vaccum.adjustFingerPosition(1, SERVO_DEBUG_STEP);
			vaccum.adjustFingerPosition(2, SERVO_DEBUG_STEP);
		}

		if (driver2.isPressed("dpad_down"))
		{
			vaccum.adjustFingerPosition(0, -SERVO_DEBUG_STEP);
			vaccum.adjustFingerPosition(1, -SERVO_DEBUG_STEP);
			vaccum.adjustFingerPosition(2, -SERVO_DEBUG_STEP);
		}

		if (driver2.justPressed("dpad_left"))
		{
			selectedDebugServo = (selectedDebugServo + 2) % 3;
		}

		if (driver2.justPressed("dpad_right"))
		{
			selectedDebugServo = (selectedDebugServo + 1) % 3;
		}
	}

	@SuppressLint("DefaultLocale") private void teleInfo ()
	{

		DebugUtil.logAdd("======== [LIME]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Target X: " + LimeUtil.getTx());
		DebugUtil.logAdd("Target D: " + LimeUtil.getTargetDistance());
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("======= [EXPLOSHER]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Target RPM: " + explosher.getTargetRPM());
		DebugUtil.logAdd("Current RPM: " + explosher.getCurrentRPM());
		DebugUtil.logAdd("Smoothed Regress: " + explosher.smoothedTargetRPM);
		DebugUtil.logAdd("Should Aim: " + (explosher.shouldAim ? "ON" : "OFF"));
		DebugUtil.logAdd("Drive Align: " + (autoAlignActive ? "ACTIVE" : "INACTIVE"));
		DebugUtil.logAdd("Exploring Pos: " + explosher.getExploringPos());
		DebugUtil.logAdd(String.format("Exploring PID: out=%.3f err=%.2f", explosher.getAimPow(), explosher.getAimErr()));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("======= [FINGER]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Finger State: " + fingerState);
		DebugUtil.logAdd("Finger Position: '" + String.format("%.3f", explosher.getFingerPosition()));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("======= [COLOR]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd(String.format(
				"Sensor2 RGB: R=%d G=%d B=%d",
				colorSensor2.red(),
				colorSensor2.green(),
				colorSensor2.blue()
		));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("======= [MEMORY]");
		DebugUtil.logAdd("[MOTIF]: " + memory.curMotif());
		DebugUtil.logAdd("[BALLS]: " + Arrays.toString(memory.curBalls()));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("[POSITION] HEADING: " + memory.curPos().getHeading());
		DebugUtil.logAdd("[POSITION] X: " + memory.curPos().getX());
		DebugUtil.logAdd("[POSITION] Y: " + memory.curPos().getY());
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("[FOLLOWER] X: " + follower.getPose().getX());
		DebugUtil.logAdd("[FOLLOWER] Y: " + follower.getPose().getY());
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("Debug Servo: " + (selectedDebugServo + 1) +
				" | Pos: " + String.format("%.3f", vaccum.getFingerPosition(selectedDebugServo)));
		DebugUtil.logAdd(" ");

		DebugUtil.update();
	}

	private boolean hasRecentTarget ()
	{

		return LimeUtil.hasValidTarget() &&
				(System.currentTimeMillis() - lastTagSeenTime) < TAG_TIMEOUT_MS;
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



