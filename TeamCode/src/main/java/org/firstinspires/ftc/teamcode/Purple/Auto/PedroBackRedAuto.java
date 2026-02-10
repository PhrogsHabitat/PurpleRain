package org.firstinspires.ftc.teamcode.Purple.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Controls;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroBackRedAuto", group = "Purple")
public class PedroBackRedAuto extends OpMode
{

	private static final long TAG_TIMEOUT_MS = 500;
	private static final double RPM_SMOOTHING_ALPHA = 0.2;
	private static final double THRESHOLD = 3;
	// sample poses (adjust to your field/layout)
	private final Pose startPose = new Pose(81, 8, Math.toRadians(90));
	private final Pose shootPose = new Pose(81, 23, Math.toRadians(63));
	private final Pose Pickup_First_Halflife1Pose = new Pose(90, 35.5, Math.toRadians(0));
	private final Pose Pickup_First_Halflife2Pose = new Pose(120, 35.5, Math.toRadians(0));
	private final Pose Pickup_Second_Halflife1Pose = new Pose(90, 60, Math.toRadians(0));
	private final Pose Pickup_Second_Halflife2Pose = new Pose(122, 60, Math.toRadians(0));
	private final Pose rankPose = new Pose(90, 35.5, Math.toRadians(90));
	public Explosher explosher;
	public Vaccum vaccum;
	public ElapsedTime shootTimer;
	public double dist;
	public boolean manual = false;
	private Follower follower;
	private double regressionSlope;
	private double regressionIntercept;
	private double smoothedTargetRPM = 0;
	private boolean shouldShoot = false;
	private Controls driver1;
	private Controls driver2;
	private MotorConfig fl, fr, bl, br;
	private double powerScale = org.firstinspires.ftc.teamcode.Purple.Constants.DRIVE_POWER_SCALE;
	private boolean wasAligned = false;
	private boolean wasTagDetected = false;
	private Explosher.FingerState fingerState = Explosher.FingerState.STOP;
	private boolean autoAlignActive = false;
	private long lastTagSeenTime = 0;
	private PurplePathing pathManager;

	@Override
	public void init ()
	{

		onCreate();
	}

	@Override
	public void loop ()
	{

		onUpdate();
	}

	private void onCreate ()
	{

		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		// create follower and purple path manager
		follower = Constants.createFollower(hardwareMap);
		follower.setPose(startPose);
		pathManager = new PurplePathing(follower);

		shootTimer = new ElapsedTime();

		calculateRegression();

		LimeUtil.start(hardwareMap, "SwagLime", 60);
		LimeUtil.setPipeline(0);
		explosher = new Explosher(hardwareMap, org.firstinspires.ftc.teamcode.Purple.Constants.FINGER_SERVO_CONFIG);
		vaccum = new Vaccum(hardwareMap);

		initializeMotors();

		// Path Chain Presets
		PathChain DriveStartShoot = follower.pathBuilder()
				.addPath(new BezierLine(startPose, shootPose))
				.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
				.build();
		PathChain DriveToHalfLife1 = follower.pathBuilder()
				.addPath(new BezierLine(shootPose, Pickup_First_Halflife1Pose))
				.setLinearHeadingInterpolation(shootPose.getHeading(), Pickup_First_Halflife1Pose.getHeading())
				.build();
		PathChain DriveHalfLife1 = follower.pathBuilder()
				.addPath(new BezierLine(Pickup_First_Halflife1Pose, Pickup_First_Halflife2Pose))
				.setLinearHeadingInterpolation(Pickup_First_Halflife1Pose.getHeading(), Pickup_First_Halflife2Pose.getHeading())
				.build();
		PathChain DrivePickupShoot1 = follower.pathBuilder()
				.addPath(new BezierLine(Pickup_First_Halflife2Pose, shootPose))
				.setLinearHeadingInterpolation(Pickup_First_Halflife2Pose.getHeading(), shootPose.getHeading())
				.build();
		PathChain DriveToHalfLife2 = follower.pathBuilder()
				.addPath(new BezierLine(shootPose, Pickup_Second_Halflife1Pose))
				.setLinearHeadingInterpolation(shootPose.getHeading(), Pickup_Second_Halflife1Pose.getHeading())
				.build();
		PathChain DriveHalfLife2 = follower.pathBuilder()
				.addPath(new BezierLine(Pickup_Second_Halflife1Pose, Pickup_Second_Halflife2Pose))
				.setLinearHeadingInterpolation(Pickup_Second_Halflife1Pose.getHeading(), Pickup_Second_Halflife2Pose.getHeading())
				.build();
		PathChain DrivePickupShoot2 = follower.pathBuilder()
				.addPath(new BezierLine(Pickup_Second_Halflife2Pose, shootPose))
				.setLinearHeadingInterpolation(Pickup_Second_Halflife2Pose.getHeading(), shootPose.getHeading())
				.build();
		PathChain RankMove = follower.pathBuilder()
				.addPath(new BezierLine(shootPose, rankPose))
				.setLinearHeadingInterpolation(shootPose.getHeading(), rankPose.getHeading())
				.build();

		// Create the PurplePath objects
		PurplePath path1 = new PurplePath("Drive Back", DriveStartShoot, 1.0, 6.5)
				.onComplete(() -> flagShoot());

		PurplePath path2 = new PurplePath("Drive back smore", DriveToHalfLife1, 1, 0.0)
				.onComplete(() -> pickupBalls(true));

		PurplePath path3 = new PurplePath("pickup BALLS =]", DriveHalfLife1, 2.0, 0)
				.onComplete(() -> pickupBalls(false));

		PurplePath path4 = new PurplePath("Drive back to shoot", DrivePickupShoot1, 1, 6.5)
				.onComplete(() -> flagShoot());

		PurplePath path5 = new PurplePath("Drive back smlot", DriveToHalfLife2, 2.0, 0.0)
				.onComplete(() -> pickupBalls(true));

		PurplePath path6 = new PurplePath("pickup BALLS =] 2: electric boogaloo", DriveHalfLife2, 1, 0.0)
				.onComplete(() -> pickupBalls(false));

		PurplePath path7 = new PurplePath("Drive Back to shoot again", DrivePickupShoot2, 2.0, 6.5)
				.onComplete(() -> flagShoot());

		PurplePath path8 = new PurplePath("Drive outta da trangle", RankMove, 1, 0.0)
				.onComplete(() -> DebugUtil.logAdd("path2 completed"));

		// Create the PurpleChain object
		PurpleChain chain = new PurpleChain(path1, path2, path3, path4, path5, path6, path7, path8)
				.onComplete(() -> DebugUtil.logAdd("Chain fully finished"));

		// Start the chain. holdEnd = true (follower will hold at end of each path)
		pathManager.startChain(chain, true, () -> DebugUtil.logAdd("startChain() provided onComplete"));

		shootTimer.reset();

		DebugUtil.setTelemetry(telemetry);
	}

	private void onUpdate ()
	{
		// update the manager (must be called every loop)
		pathManager.update();
		explosher.update();
		vaccum.update();
		updateAllSystems();

		if (shouldShoot)
		{
			shootFull();
		}

		// logging
		DebugUtil.logAdd("Current Path: " + pathManager.curPath());
		DebugUtil.update();
	}

	private void flagShoot ()
	{

		shouldShoot = true;
		shootTimer.reset();
		shootTimer.startTime();
	}

	private void pickupBalls (boolean should)
	{

		exploSwag(false, "forward");
		toggleY(should);
		explosher.setFingerState(Explosher.FingerState.STOP);
	}

	private void shootFull ()
	{
		// Start Stuff

		if (shootTimer.seconds() <= 0)
		{
			autoAlignActive = true;
			explosher.setFingerState(Explosher.FingerState.STOP);
		}
		if (shootTimer.seconds() > 2)
		{
			explosher.setFingerState(Explosher.FingerState.PASS);
			autoAlignActive = false;
		}
		if (shootTimer.seconds() > 0 && shootTimer.seconds() < 6.5)
		{
			exploSwag(true, "Forward");
		}
		if (shootTimer.seconds() > 3.5 && shootTimer.seconds() < 3.6)
		{
			toggleY(true);
			DebugUtil.logAdd("first push on");
		}
		if (shootTimer.seconds() > 3.6 && shootTimer.seconds() < 4.1)
		{
			toggleY(false);
			DebugUtil.logAdd("first push off");
		}
		if (shootTimer.seconds() > 5 && shootTimer.seconds() < 5.5)
		{
			toggleY(true);
			DebugUtil.logAdd("first push on");
		}
		if (shootTimer.seconds() == 5.5)
		{
			toggleY(false);
			DebugUtil.logAdd("first push off");
		}
		if (shootTimer.seconds() > 5.5 && shootTimer.seconds() < 6)
		{
			toggleX(true);
		}
		if (shootTimer.seconds() == 6)
		{
			toggleX(false);
		}
		if (shootTimer.seconds() > 6 && shootTimer.seconds() < 6.5)
		{
			toggleY(true);
		}
		if (shootTimer.seconds() >= 6.5)
		{
			toggleY(false);
			exploSwag(false, "Forward");
			shouldShoot = false;
			explosher.setFingerState(Explosher.FingerState.STOP);
		}
	}

	public void exploSwag (boolean should, String Explostate)
	{

		if (should)
		{
			if (Explostate == "Forward")
			{
				if (LimeUtil.getTargetDistance() != 0)
				{
					dist = LimeUtil.getTargetDistance();
					double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;
					smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
					smoothedTargetRPM = Math.max(0, Math.min(smoothedTargetRPM, explosher.getMaxRPM()));
					explosher.setRPM(smoothedTargetRPM);
				}
			} else if (Explostate == "Back")
			{
				explosher.setRPM(-4000);
			}
		} else
		{
			explosher.stop();
		}
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

	private void calculateRegression ()
	{

		double[][] calibrationPoints = {
				{59, 2700},
				{65, 2800},
				{77, 2850},
				{80, 3000},
				{94, 2850}
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
			driver2.vibrate(org.firstinspires.ftc.teamcode.Purple.Constants.VIBRATION_TAG_DETECTED);
		}

		wasTagDetected = tagDetected;
	}

//    private void updateSubsystems ()
//    {
//
//        explosher.update();
//        vaccum.update();
//    }

	private void updateDebug ()
	{

		DebugUtil.logAdd("Finger State: " + fingerState);
		DebugUtil.logAdd("Finger Position: " + String.format("%.3f", explosher.getFingerPosition()));
	}

	private void updateDrive (String dir)
	{

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

	private boolean isFullyAligned ()
	{

		if (!hasRecentTarget()) return false;

		double tx = LimeUtil.getTx();
		double distance = LimeUtil.getTargetDistance();
		double distanceError = Math.abs(distance - org.firstinspires.ftc.teamcode.Purple.Constants.DESIRED_TAG_DISTANCE);

		return Math.abs(tx) < org.firstinspires.ftc.teamcode.Purple.Constants.ALIGN_ANGLE_TOLERANCE &&
				distanceError < org.firstinspires.ftc.teamcode.Purple.Constants.ALIGN_DISTANCE_TOLERANCE;
	}

	private boolean hasRecentTarget ()
	{

		return LimeUtil.hasValidTarget() &&
				(System.currentTimeMillis() - lastTagSeenTime) < TAG_TIMEOUT_MS;
	}

//    private void updateExplosherControl ()
//    {
//
//        double leftStickY = driver2.getLeftStickY();
//
//        if (leftStickY > org.firstinspires.ftc.teamcode.Purple.Constants.JOYSTICK_DEADZONE)
//        {
//            if (LimeUtil.getTargetDistance() != 0)
//            {
//                dist = LimeUtil.getTargetDistance();
//                double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;
//                smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
//                smoothedTargetRPM = Math.max(0, Math.min(smoothedTargetRPM, explosher.getMaxRPM()));
//
//                if (!manual)
//                {
//                    explosher.setRPM(smoothedTargetRPM);
//                }
//            } else if (!manual)
//            {
//                explosher.stop();
//            }
//        } else if (leftStickY < -org.firstinspires.ftc.teamcode.Purple.Constants.JOYSTICK_DEADZONE && driver2.isPressed("x"))
//        {
//            explosher.setRPM(-4000);
//            vaccum.setPower(-Vaccum.DEFAULT_POW);
//        } else if (driver2.isPressed("b"))
//        {
//            explosher.setRPM(-4000);
//            vaccum.swagReverse(-Vaccum.DEFAULT_POW);
//        } else
//        {
//            explosher.stop();
//        }
//
//        if (driver2.isPressed("x"))
//        {
//            vaccum.setPower(-Vaccum.DEFAULT_POW);
//        }
//    }

	private void toggleY (boolean should)
	{

		double pow = should ? Vaccum.DEFAULT_POW : 0;
		vaccum.setPower(pow);
	}

	private void toggleX (boolean should)
	{

		double pow = should ? -.15 : 0;
		vaccum.setPower(pow);
	}

	private void toggleSuperX (boolean should)
	{

		double rpm = should ? -4000 : 0.0;
		double pow = should ? -Vaccum.DEFAULT_POW : 0;
		exploSwag(should, should ? "Back" : "Off");
		vaccum.setPower(pow);
	}

	private void toggleB (boolean should)
	{

		double rpm = should ? -4000 : 0.0;
		double pow = should ? -.50 : 0;
		exploSwag(should, should ? "Back" : "Off");
	}

	private void updateAllSystems ()
	{

		LimeUtil.update();
		updateAprilTagFeedback();
//        updateSubsystems();

		if (org.firstinspires.ftc.teamcode.Purple.Constants.DEBUG_MODE)
		{
			updateDebug();
		}

		if (autoAlignActive)
		{
			if (Math.abs(LimeUtil.getTx()) > THRESHOLD)
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

//        updateExplosherControl();
		updateTelemetry();
	}
}