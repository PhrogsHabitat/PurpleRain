package org.firstinspires.ftc.teamcode.Purple.Extras;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Controls;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TempOp", group = "Purple")
public class TempOp extends PurpleOpMode
{
	private static final long TAG_TIMEOUT_MS = 500;
	private static final double RPM_SMOOTHING_ALPHA = 0.2;
	private static final double THRESHOLD = 3;

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
	private boolean tagDetected = false;
	private Explosher.FingerState fingerState = Explosher.FingerState.STOP;
	private boolean autoAlignActive = false;
	private long lastTagSeenTime = 0;
	private double smoothedTargetRPM = 0;

	// Core methods ALWAYS come first (excluding destroy)
	@Override
	public void create ()
	{

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
		// LimeUtil.start(hardwareMap, "SwagLime", 60);
		// LimeUtil.setPipeline(0);

		// Initialize Explosher
		// explosher = new Explosher(hardwareMap, Constants.FINGER_SERVO_CONFIG);

		// Initialize Vaccum
		vaccum = new Vaccum(hardwareMap);

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update ()
	{

		// Update the controller of each driver
		driver1.update();
		driver2.update();

		// Call the mini-updates
//        LimeUtil.update();
		DebugUtil.update();
//        explosher.update();
		vaccum.update();

		// Update each control-based module
//        updateExplosher();
		updateVaccum();

		updateHaptics();
		updateDebug();
		teleInfo();

		// Store weather we see a tag or not
		// tagDetected = LimeUtil.hasValidTarget();

//        autoAlignActive = driver1.isPressed("right_bumper") && tagDetected;
//        wasTagDetected = tagDetected;
//        wasAligned = isFullyAligned();

		// Update the Drive
//        if (autoAlignActive)
//        {
//            if (Math.abs(LimeUtil.getTx()) > THRESHOLD)
//            {
//                if(LimeUtil.getTx() < 0)
//                {
//                    updateDrive("L");
//                }
//                else if(LimeUtil.getTx() > 0)
//                {
//                    updateDrive("R");
//                }
//                else
//                {
//                    updateDrive("def");
//                }
//            }
//            else
//            {
//                driver1.vibrate(150);
//                updateDrive("def");
//            }
//        }

		updateDrive("def");

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
		} else if (dir == "R")
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
		} else
		{
			double forward = driver1.getLeftStickY();
			double strafe = driver1.getLeftStickX();
			double turn = driver1.getRightStickX();

			double[] rawPowers = new double[]{
					(-forward - strafe - turn),
					(-forward + strafe - turn),
					(forward - strafe - turn),
					(forward + strafe - turn)
			};

			double[] normPowers = MotorUtil.normalizePowers(new double[]{
					(-forward - strafe - turn),
					(-forward + strafe - turn),
					(forward - strafe - turn),
					(forward + strafe - turn)
			});

			double[] powers = driver1.isPressed("left_stick_button") ? rawPowers : normPowers;

			fl.setPower(powers[0] * powerScale);
			bl.setPower(powers[1] * powerScale);
			fr.setPower(powers[2] * powerScale);
			br.setPower(powers[3] * powerScale);
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
		} else if (driver2.isPressed("b"))
		{
			explosher.setRPM(-4000);
		} else
		{
			explosher.stop();
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
	}

	private void updateVaccum ()
	{

		if (driver2.isPressed("y"))
		{
			vaccum.setPower(Vaccum.DEFAULT_POW + 5.0);
		} else if (driver2.isPressed("x"))
		{
			// This one needs to be slower cuz its too fast
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		} else
		{
			vaccum.stop();
		}
	}

	public void updateHaptics ()
	{

		if (tagDetected)
		{
			lastTagSeenTime = System.currentTimeMillis();
		}

		if (tagDetected && !wasTagDetected)
		{
			driver1.vibrate(150);
			driver2.vibrate(Constants.VIBRATION_TAG_DETECTED);
		}

		if (isFullyAligned() && !wasAligned)
		{
			driver2.vibrate(Constants.VIBRATION_ALIGNED);
		}
	}

	public void updateDebug ()
	{

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

	private void teleInfo ()
	{

		DebugUtil.logAdd("TX: " + LimeUtil.getTx());
		DebugUtil.logAdd("Target Distance: " + LimeUtil.getTargetDistance());
		DebugUtil.logAdd("Explosher Target RPM: " + String.format("%.1f", explosher.getTargetRPM()));
		DebugUtil.logAdd("Explosher Current RPM: " + String.format("%.1f", explosher.getCurrentRPM()));
		DebugUtil.logAdd("Vaccum Current Power: " + vaccum.getPower());
		DebugUtil.logAdd("FR: " + fr.getPower());
		DebugUtil.logAdd("FL: " + fl.getPower());
		DebugUtil.logAdd("BR: " + br.getPower());
		DebugUtil.logAdd("BL: " + bl.getPower());

		if (LimeUtil.hasValidTarget())
		{
			DebugUtil.logAdd("AprilTag - Dist: " + String.format("%.1f", LimeUtil.getTargetDistance()) +
					"in, Angle: " + String.format("%.1f", LimeUtil.getTx()) + "°");
			DebugUtil.logAdd("Aligned: " + (isFullyAligned() ? "YES" : "NO"));
		} else
		{
			DebugUtil.logAdd("AprilTag: No target");
		}
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

//{57, 2500},
//{80, 2700},
//{95, 2900},
//{110, 2900},
//{130, 3100}