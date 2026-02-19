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
import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
public class TeleOp extends PurpleOpMode
{
	// Private variables after
	private static final long TAG_TIMEOUT_MS = 500;
	private static final double DRIVE_ALIGN_TX_THRESHOLD = 3.0;
	private static final double TURRET_ALIGN_TX_DEADBAND = 1.0;
	private static final double SERVO_DEBUG_STEP = 0.01;
	private static final double TURRET_PID_KP = 0.036;
	private static final double TURRET_PID_KI = 0.0012;
	private static final double TURRET_PID_KD = 0.0020;
	private static final double TURRET_PID_DEADBAND = 0.3;
	private static final double TURRET_PID_MAX_POWER = 1.0;
	private static final double TURRET_PID_MAX_SLEW_PER_SEC = 6.0;
	private static final double TURRET_PID_INTEGRAL_LIMIT = 35.0;
	private static final double TURRET_PID_DERIVATIVE_ALPHA = 0.2;
	private static final double TURRET_FLIP_BOOST_ERROR_DEG = 80.0;
	private static final double TURRET_FLIP_BOOST_POWER = 1.0;
	private static final double TURRET_FLIP_BOOST_SLEW_PER_SEC = 14.0;
	private static final double TURRET_ALIGN_MAX_STEP = 0.12;
	private static final double TURRET_MIN_ANGLE_DEG = -180.0;
	private static final double TURRET_MAX_ANGLE_DEG = 180.0;
	// Public variables first
	public static Pose startingPose;
	public double turretIncrement = 2500;
	public double swagShitFar = Explosher.FAR_SWEET;
	public double dist;
	public boolean manual = false;
	private int inc = 0;
	private double lastPower = 0;
	private double ALIGN_KP = 0.04;
	private double prevX;
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
	private double smoothedTargetRPM = 0;
	private int selectedDebugServo = 0;
	private double turretPidIntegral = 0;
	private double turretPidPrevError = 0;
	private double turretPidFilteredDerivative = 0;
	private double turretPidOutput = 0;
	private long turretPidPrevTimeNs = 0;

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

		boolean busy = false;
		if (autoAlignActive)
		{
			busy = true;
			double tx = LimeUtil.getTx();

			if (LimeUtil.hasValidTarget())
			{
				prevX = tx;

				if (Math.abs(tx) > TURRET_ALIGN_TX_DEADBAND)
				{
					double targetPower = MathUtil.clamp(ALIGN_KP * tx, -1, 1);

					double power = MathUtil.clamp(
							targetPower,
							lastPower - TURRET_ALIGN_MAX_STEP,
							lastPower + TURRET_ALIGN_MAX_STEP
					);

					lastPower = power;
					explosher.setRingPower(power);
				} else
				{
					explosher.setRingPower(0);
				}
			} else if (hasRecentTarget())
			{
				double power = MathUtil.clamp(prevX * ALIGN_KP * 0.5, -1, 1);
				explosher.setRingPower(power);
			} else
			{
				explosher.setRingPower(0);
			}

		} else
		{
			// Manual control
			double rightStickX = driver2.getRightStickX();
			if (Math.abs(rightStickX) > Constants.JOYSTICK_DEADZONE)
			{
				busy = true;
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

		if (driver2.isPressed("a"))
		{
			explosher.resetRingPosition();
		}

		if (driver2.justPressed("left_trigger"))
		{
			inc -= 20;
		}

		if (driver2.isPressed("b"))
		{
			explosher.setRingPosition(inc, 0.5);
			inc += 5;
		}

		if (!busy)
		{
			double targetX = 128;
			double targetY = 130;
			double robotX = follower.getPose().getX();
			double robotY = follower.getPose().getY();
			double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());

			double targetBearingDeg = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX));
			double turretTargetDeg = getSafeTurretTargetDeg(targetBearingDeg + robotHeadingDeg);
			double turretPower = getTurretPidPower(turretTargetDeg);

			explosher.setRingPower(turretPower);

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
		DebugUtil.logAdd("Auto-Align: " + (autoAlignActive ? "ACTIVE" : "INACTIVE"));
		DebugUtil.logAdd("Ring Position: " + explosher.getRingPosition());
		DebugUtil.logAdd(String.format("Turret PID: out=%.3f err=%.2f", turretPidOutput, turretPidPrevError));
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

	private double getTurretPidPower (double targetAngleDeg)
	{

		double currentAngleDeg = explosher.getAngle();
		double clampedTargetAngleDeg = MathUtil.clamp(targetAngleDeg, TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);
		double error = clampedTargetAngleDeg - currentAngleDeg;

		long nowNs = System.nanoTime();
		double dt = turretPidPrevTimeNs == 0 ? 0.02 : (nowNs - turretPidPrevTimeNs) / 1_000_000_000.0;
		turretPidPrevTimeNs = nowNs;
		dt = MathUtil.clamp(dt, 0.001, 0.1);

		if (Math.abs(error) < TURRET_PID_DEADBAND)
		{
			turretPidIntegral = 0;
			turretPidFilteredDerivative = 0;
			turretPidPrevError = error;
			turretPidOutput = slewPower(turretPidOutput, 0, dt);
			return turretPidOutput;
		}

		if (Math.abs(error) >= TURRET_FLIP_BOOST_ERROR_DEG)
		{
			turretPidIntegral = 0;
			double boostPower = Math.copySign(TURRET_FLIP_BOOST_POWER, error);
			boostPower = applyTurretHardStop(currentAngleDeg, boostPower);
			turretPidOutput = slewPower(turretPidOutput, boostPower, dt, TURRET_FLIP_BOOST_SLEW_PER_SEC);
			turretPidPrevError = error;
			return turretPidOutput;
		}

		turretPidIntegral += error * dt;
		turretPidIntegral = MathUtil.clamp(turretPidIntegral, -TURRET_PID_INTEGRAL_LIMIT, TURRET_PID_INTEGRAL_LIMIT);

		double rawDerivative = (error - turretPidPrevError) / dt;
		turretPidFilteredDerivative += TURRET_PID_DERIVATIVE_ALPHA * (rawDerivative - turretPidFilteredDerivative);

		double targetPower = (TURRET_PID_KP * error) +
				(TURRET_PID_KI * turretPidIntegral) +
				(TURRET_PID_KD * turretPidFilteredDerivative);
		targetPower = MathUtil.clamp(targetPower, -TURRET_PID_MAX_POWER, TURRET_PID_MAX_POWER);
		targetPower = applyTurretHardStop(currentAngleDeg, targetPower);

		turretPidOutput = slewPower(turretPidOutput, targetPower, dt);
		turretPidPrevError = error;
		return turretPidOutput;
	}

	private double slewPower (double current, double target, double dt)
	{
		return slewPower(current, target, dt, TURRET_PID_MAX_SLEW_PER_SEC);
	}

	private double slewPower (double current, double target, double dt, double maxSlewPerSec)
	{

		double maxStep = maxSlewPerSec * dt;
		return MathUtil.clamp(target, current - maxStep, current + maxStep);
	}

	private double normalizeAngleDegrees (double angleDeg)
	{

		while (angleDeg > 180)
		{
			angleDeg -= 360;
		}

		while (angleDeg < -180)
		{
			angleDeg += 360;
		}

		return angleDeg;
	}

	private double getSafeTurretTargetDeg (double desiredAngleDeg)
	{

		return MathUtil.clamp(
				normalizeAngleDegrees(desiredAngleDeg),
				TURRET_MIN_ANGLE_DEG,
				TURRET_MAX_ANGLE_DEG
		);
	}

	private double applyTurretHardStop (double currentAngleDeg, double requestedPower)
	{

		if (currentAngleDeg >= TURRET_MAX_ANGLE_DEG && requestedPower > 0)
		{
			return 0;
		}

		if (currentAngleDeg <= TURRET_MIN_ANGLE_DEG && requestedPower < 0)
		{
			return 0;
		}

		return requestedPower;
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



