//package org.firstinspires.ftc.teamcode.Purple.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
//import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
//import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
//import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;
//import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
//import org.firstinspires.ftc.teamcode.Purple.Constants;
//import org.firstinspires.ftc.teamcode.Purple.Controls;
//import org.firstinspires.ftc.teamcode.Purple.Names;
//import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueAuto", group = "Purple")
//public class BlueAuto extends LinearOpMode
//{
//	private static final long TAG_TIMEOUT_MS = 500;
//	private final double powerScale = Constants.DRIVE_POWER_SCALE;
//	private final ElapsedTime timer = new ElapsedTime();
//	// Use Explosher presets by default (Option A)
//	public double swagShitClose = Explosher.CLOSE_SWEET; // 900
//	public double swagShitFar = Explosher.FAR_SWEET;   // 1600
//	private Controls driver1;
//	private Controls driver2;
//	private MotorConfig fl, fr, bl, br;
//	private Explosher explosher;
//	private Vaccum vaccum;
//	// State tracking
//	private boolean wasAligned = false;
//	private boolean wasTagDetected = false;
//	private Explosher.DistanceState stickyState = null;
//	// Auto-align state
//	private boolean autoAlignActive = false;
//	private long lastTagSeenTime = 0;
//
//	@Override
//	public void runOpMode ()
//	{
//		driver1 = new Controls(gamepad1);
//		driver2 = new Controls(gamepad2);
//
//		initializeMotors();
//		initializeExplosher();
//		initializeVaccum();
//		DebugUtil.setTelemetry(telemetry);
//
//		waitForStart();
//
//		timer.reset();
//		while (opModeIsActive())
//		{
//            driver1.update();
//            driver2.update();
//			update();
//		}
//		stopAll();
//	}
//
//	private void initializeMotors ()
//	{
//		// Keep your mecanum and motor parameters as they were
//		fl = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.FRONT_LEFT, 2150.8, 312).build();
//		fr = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.FRONT_RIGHT, 2150.8, 312).build();
//		bl = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.BACK_LEFT, 2150.8, 312).build();
//		br = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.BACK_RIGHT, 2150.8, 312).build();
//	}
//
//	private void initializeExplosher ()
//	{
//		// Lime camera + explosher init (unchanged)
//		LimeUtil.start(hardwareMap, "SwagLime", 60);
//		LimeUtil.setPipeline(0);
//		explosher = new Explosher(hardwareMap, Constants.FINGER_SERVO_CONFIG);
//	}
//
//	private void initializeVaccum ()
//	{
//		vaccum = new Vaccum(hardwareMap);
//	}
//
//	private void update ()
//	{
//		updateAprilTagFeedback();
////        updatePlayer1Controls();
////        updatePlayer2Controls(); // consolidated operator controls
//
//		// Only run debug adjustments once per loop if enabled
//		if (Constants.DEBUG_MODE)
//		{
//			updateDebug();
//		}
//
//		if (autoAlignActive)
//		{
//			autoAlignToTag();
//		} else
//		{
//			updateDrive();
//		}
//
//		updateSubsystems();
//		updateTelemetry();
//	}
//
//	// ===== DRIVE (unchanged directions & math) =====
//
//	private void updateDrive ()
//	{
//
//		// Start the elapsed timer :3
//		double elapsed = timer.seconds();
//
//		// Speed boost when left stick button is pressed
//		// powerScale = driver1.isPressed("left_stick_button") ? Constants.DRIVE_POWER_BOOST : Constants.DRIVE_POWER_SCALE;
//
//		double forward = 0;
//		double strafe = 0;
//		double turn = 0;
//
//		if (elapsed < 2.25)
//		{
//			forward = -0.4;
//		}
//
//		if (elapsed > 3)
//		{
//			explosher.setMotorState(MotorConfig.MotorState.ON);
//			explosher.setRPM(Explosher.CLOSE_SWEET);
//			autoAlignToTag();
//		}
//
//		if (elapsed > 6 && elapsed < 16)
//		{
//			autoAlignActive = false;
//			vaccum.setState(MotorConfig.MotorState.ON);
//		}
//
//		if (elapsed > 16 && elapsed < 18)
//		{
//			strafe = 0.5;
//			vaccum.setState(MotorConfig.MotorState.OFF);
//			explosher.setMotorState(MotorConfig.MotorState.OFF);
//		}
//
//		if (elapsed > 18)
//		{
//			forward = 0;
//			strafe = 0;
//			turn = 0;
//
//			vaccum.setState(MotorConfig.MotorState.OFF);
//			explosher.setMotorState(MotorConfig.MotorState.OFF);
//		}
//
//		double[] powers = MotorUtil.normalizePowers(new double[]{
//				(-forward - strafe - turn),
//				(-forward + strafe - turn),
//				(forward - strafe - turn),
//				(forward + strafe - turn)
//		});
//
//		fl.setPower(powers[0] * powerScale);
//		bl.setPower(powers[1] * powerScale);
//		fr.setPower(powers[2] * powerScale);
//		br.setPower(powers[3] * powerScale);
//	}
//
//	// ===== APRILTAG / TAG FEEDBACK =====
//
//	private void updateAprilTagFeedback ()
//	{
//		boolean tagDetected = LimeUtil.hasValidTarget();
//
//		if (tagDetected)
//		{
//			lastTagSeenTime = System.currentTimeMillis();
//		}
//
//		// Quick vibration when tag newly detected
//		if (tagDetected && !wasTagDetected)
//		{
//			driver1.vibrate(Constants.VIBRATION_TAG_DETECTED);
//			driver2.vibrate(Constants.VIBRATION_TAG_DETECTED);
//		}
//
//		wasTagDetected = tagDetected;
//	}
//
//	// ===== PLAYER 1 (driver) =====
//
//	private void updatePlayer1Controls ()
//	{
//		// Auto-align with AprilTag when right bumper held and tag present
//		autoAlignActive = driver1.isPressed("right_bumper") && LimeUtil.hasValidTarget();
//
//		// Impact detection placeholder
//		checkForImpact();
//	}
//
//	// ===== PLAYER 2 (operator) =====
//
//	private void updatePlayer2Controls ()
//	{
//		// --- Explosher stick behavior (temporary manual control) ---
//		double leftStickY = driver2.getLeftStickY();
//
//		if (Math.abs(leftStickY) > Constants.JOYSTICK_DEADZONE)
//		{
//			// Stick being used: clear sticky state (temporary manual control)
//			stickyState = null;
//
//			if (driver2.isPressed("left_stick_button"))
//			{
//				// Stick pressed + forward => FAR sweet spot
//				explosher.setRPM(swagShitFar);
//				explosher.setMotorState(MotorConfig.MotorState.ON);
//			} else if (leftStickY > 0.5)
//			{
//				// Stick forward => CLOSE sweet spot
//				explosher.setRPM(swagShitClose);
//				explosher.setMotorState(MotorConfig.MotorState.ON);
//			}
//			// NOTE: if stick used but not forwarded >0.5, we don't change RPM (keep previous)
//		} else if (stickyState == null)
//		{
//			// Stick returned to center and no sticky preset => turn off
//			explosher.setMotorState(MotorConfig.MotorState.OFF);
//		}
//
//		// --- Explosher trigger-based sticky toggle (same behavior) ---
//		if (driver2.justPressed("left_trigger"))
//		{
//			if (stickyState == null || stickyState == Explosher.DistanceState.FAR)
//			{
//				stickyState = Explosher.DistanceState.NEAR;
//				explosher.setRPM(Explosher.CLOSE_SWEET);
//			} else
//			{
//				stickyState = Explosher.DistanceState.FAR;
//				explosher.setRPM(Explosher.FAR_SWEET);
//			}
//			explosher.setMotorState(MotorConfig.MotorState.ON);
//		}
//
//		// --- Vacuum control (operator) ---
//		if (driver2.isPressed("y"))
//		{
//			vaccum.setState(MotorConfig.MotorState.ON);
//		} else if (driver2.isPressed("x"))
//		{
//			vaccum.setState(MotorConfig.MotorState.ON);
//			vaccum.setPower(-Vaccum.DEFAULT_POW);
//		} else
//		{
//			vaccum.setState(MotorConfig.MotorState.OFF);
//		}
//
//		// --- Debug dpad tweaks (keeps original behavior) ---
//		// NOTE: updateDebug() is called once per loop if DEBUG_MODE is true;
//		// we still allow driver2 to bump sweet spots here for quick tuning too.
//		if (driver2.justPressed("dpad_up") && Constants.DEBUG_MODE)
//		{
//			swagShitClose += 100;
//		}
//		if (driver2.justPressed("dpad_down") && Constants.DEBUG_MODE)
//		{
//			swagShitClose -= 100;
//		}
//		if (driver2.justPressed("dpad_left") && Constants.DEBUG_MODE)
//		{
//			swagShitFar += 100;
//		}
//		if (driver2.justPressed("dpad_right") && Constants.DEBUG_MODE)
//		{
//			swagShitFar -= 100;
//		}
//
//		// --- Alignment vibration feedback for operator ---
//		if (isFullyAligned() && !wasAligned)
//		{
//			driver2.vibrate(Constants.VIBRATION_ALIGNED);
//		}
//		wasAligned = isFullyAligned();
//	}
//
//	private void updateDebug ()
//	{
//		// Keep this available for general debug toggles if DEBUG_MODE is enabled.
//		// (We already expose dpad adjustments inside updatePlayer2Controls for quick tuning.)
//	}
//
//	// ===== AUTO-ALIGN HELPERS =====
//
//	private boolean hasRecentTarget ()
//	{
//		return LimeUtil.hasValidTarget() &&
//				(System.currentTimeMillis() - lastTagSeenTime) < TAG_TIMEOUT_MS;
//	}
//
//	private void autoAlignToTag ()
//	{
//		if (!hasRecentTarget())
//		{
//			autoAlignActive = false;
//			return;
//		}
//
//		double tx = LimeUtil.getTx(); // horizontal offset (deg)
//		double distance = LimeUtil.getTargetDistance();
//
//		// errors
//		double angleError = -tx;
//		double distanceError = Constants.DESIRED_TAG_DISTANCE - distance;
//
//		// apply deadzone
//		if (Math.abs(angleError) < Constants.ALIGN_ANGLE_DEADZONE) angleError = 0;
//		if (Math.abs(distanceError) < Constants.ALIGN_DISTANCE_DEADZONE) distanceError = 0;
//
//		double strafePower = clamp(angleError * Constants.ALIGN_ANGLE_KP,
//				-Constants.MAX_ALIGN_POWER, Constants.MAX_ALIGN_POWER);
//
//		double forwardPower = 0; // intentionally 0 for angle-only strafing
//		double turnPower = clamp(-angleError * Constants.ALIGN_ANGLE_KP,
//				-Constants.MAX_ALIGN_POWER, Constants.MAX_ALIGN_POWER);
//
//		double[] powers = MotorUtil.normalizePowers(new double[]{
//				(-forwardPower - strafePower - turnPower),
//				(-forwardPower + strafePower - turnPower),
//				(forwardPower - strafePower - turnPower),
//				(forwardPower + strafePower - turnPower)
//		});
//
//		fl.setPower(powers[0] * powerScale);
//		bl.setPower(powers[1] * powerScale);
//		fr.setPower(powers[2] * powerScale);
//		br.setPower(powers[3] * powerScale);
//
//		// vibration feedback
//		double alignmentError = Math.abs(angleError) + Math.abs(distanceError);
//		if (alignmentError < 2.0)
//		{
//			driver1.vibrate(50); // continuous gentle
//		} else if (alignmentError < 5.0)
//		{
//			if ((System.currentTimeMillis() % 500) < 250)
//			{
//				driver1.vibrate(25); // pulsed
//			}
//		}
//	}
//
//	private double clamp (double value, double min, double max)
//	{
//		return Math.max(min, Math.min(max, value));
//	}
//
//	private boolean isFullyAligned ()
//	{
//		if (!hasRecentTarget()) return false;
//
//		double tx = LimeUtil.getTx();
//		double distance = LimeUtil.getTargetDistance();
//		double distanceError = Math.abs(distance - Constants.DESIRED_TAG_DISTANCE);
//
//		return Math.abs(tx) < Constants.ALIGN_ANGLE_TOLERANCE &&
//				distanceError < Constants.ALIGN_DISTANCE_TOLERANCE;
//	}
//
//	private void checkForImpact ()
//	{
//		// Placeholder - leave for IMU/encoders implementation
//	}
//
//	// ===== SUBSYSTEM UPDATES =====
//
//	private void updateSubsystems ()
//	{
//		explosher.update();
//		vaccum.update();
//	}
//
//	// ===== TELEMETRY =====
//
//	private void updateTelemetry ()
//	{
//		DebugUtil.logAdd("Power Scale: " + powerScale);
//		DebugUtil.logAdd("Auto-Align: " + (autoAlignActive ? "ACTIVE" : "INACTIVE"));
//		DebugUtil.logAdd("Sticky State: " + stickyState);
//
//		if (LimeUtil.hasValidTarget())
//		{
//			DebugUtil.logAdd("AprilTag - Dist: " + String.format("%.1f", LimeUtil.getTargetDistance()) +
//					"in, Angle: " + String.format("%.1f", LimeUtil.getTx()) + "°");
//			DebugUtil.logAdd("Aligned: " + (isFullyAligned() ? "YES" : "NO"));
//		} else
//		{
//			DebugUtil.logAdd("AprilTag: No target");
//		}
//
//		DebugUtil.logAdd("Explosher RPM: " + String.format("%.1f", explosher.getCurrentRPM()));
//		DebugUtil.logAdd("Explosher Target RPM: " + String.format("%.1f", explosher.getTargetRPM()));
//		DebugUtil.update();
//	}
//
//	private void stopAll ()
//	{
//		fl.stop();
//		fr.stop();
//		bl.stop();
//		br.stop();
//		explosher.setMotorState(MotorConfig.MotorState.OFF);
//		vaccum.setState(MotorConfig.MotorState.OFF);
//		autoAlignActive = false;
//	}
//}
