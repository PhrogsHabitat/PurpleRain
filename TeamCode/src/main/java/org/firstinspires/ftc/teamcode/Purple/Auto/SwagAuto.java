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
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "SwagAuto", group = "Purple")
public class SwagAuto extends OpMode
{
	// Private variables after
	private static final double RPM_SMOOTHING_ALPHA = 0.2;
	// Pose constants
	private final Pose startPose = new Pose(21.28301886792453, 123.84905660377358, Math.toRadians(143));
	private final Pose shootPose = new Pose(53.43396226415094, 94.41509433962264, Math.toRadians(143));
	private final Pose Pickup_First_Halflife1Pose = new Pose(53.440993788819874, 83.0323509898277, Math.toRadians(180));
	private final Pose Pickup_First_Halflife2Pose = new Pose(25, 83.0323509898277, Math.toRadians(180));
	private final Pose Pickup_Second_Halflife1Pose = new Pose(48.014815154531284, 57.12668327854573, Math.toRadians(180));
	private final Pose Pickup_Second_Halflife2Pose = new Pose(25, 59.13660577338656, Math.toRadians(180));
	private final Pose rankPose = new Pose(39.751800453518925, 62.93312604141928, Math.toRadians(90));
	// Public variables first
	public Explosher explosher;
	public Vaccum vaccum;
	public ElapsedTime shootTimer;
	public double dist;
	private Follower follower;
	private double regressionSlope;
	private double regressionIntercept;
	private double smoothedTargetRPM = 0;
	private boolean shouldShoot = false;
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

	// Public utility methods with documentation

	/**
	 * Controls the explosher based on distance and regression calculation
	 *
	 * @param should         Whether the explosher should be active
	 * @param explosherState State of the explosher ("Forward" or "Back")
	 */
	public void exploSwag (boolean should, String explosherState)
	{

		if (should)
		{
			if (explosherState.equals("Forward"))
			{
				if (LimeUtil.getTargetDistance() != 0)
				{
					dist = LimeUtil.getTargetDistance();
					double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;
					smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
					smoothedTargetRPM = Math.max(0, Math.min(smoothedTargetRPM, explosher.getMaxRPM()));
					explosher.setRPM(smoothedTargetRPM);
				}
			} else if (explosherState.equals("Back"))
			{
				explosher.setRPM(-4000);
			}
		} else
		{
			explosher.stop();
		}
	}

	private void onCreate ()
	{

		follower = Constants.createFollower(hardwareMap);
		follower.setPose(startPose);
		pathManager = new PurplePathing(follower);

		shootTimer = new ElapsedTime();
		calculateRegression();

		LimeUtil.start(hardwareMap, "SwagLime", 60);
		LimeUtil.setPipeline(0);
		explosher = new Explosher(hardwareMap, org.firstinspires.ftc.teamcode.Purple.Constants.FINGER_SERVO_CONFIG);
		vaccum = new Vaccum(hardwareMap);

		setupPaths();
		shootTimer.reset();
		DebugUtil.setTelemetry(telemetry);
	}

	private void setupPaths ()
	{

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

		PurplePath path1 = new PurplePath("Drive Back", DriveStartShoot, 2.0, 8.0)
				.onComplete(() -> flagShoot());
		PurplePath path2 = new PurplePath("Drive back smore", DriveToHalfLife1, 1.5, 0.0)
				.onComplete(() -> pickupBalls(true));
		PurplePath path3 = new PurplePath("pickup BALLS =]", DriveHalfLife1, 2.0, 2.0)
				.onComplete(() -> pickupBalls(false));
		PurplePath path4 = new PurplePath("Drive back to shoot", DrivePickupShoot1, 1.5, 8.0)
				.onComplete(() -> flagShoot());
		PurplePath path5 = new PurplePath("Drive back smlot", DriveToHalfLife2, 2.0, 2.0)
				.onComplete(() -> pickupBalls(true));
		PurplePath path6 = new PurplePath("pickup BALLS =] 2: electric boogaloo", DriveHalfLife2, 1.5, 0.0)
				.onComplete(() -> pickupBalls(false));
		PurplePath path7 = new PurplePath("Drive Back to shoot again", DrivePickupShoot2, 2.0, 2.0)
				.onComplete(() -> flagShoot());
		PurplePath path8 = new PurplePath("Drive outta da trangle", RankMove, 1.5, 0.0)
				.onComplete(() -> DebugUtil.logAdd("path2 completed"));

		PurpleChain chain = new PurpleChain(path1, path2, path3, path4, path5, path6, path7, path8)
				.onComplete(() -> DebugUtil.logAdd("Chain fully finished"));

		pathManager.startChain(chain, true, () -> DebugUtil.logAdd("startChain() provided onComplete"));
	}

	private void onUpdate ()
	{

		pathManager.update();
		explosher.update();
		vaccum.update();

		if (shouldShoot)
		{
			shootFull();
		}

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

		double seconds = shootTimer.seconds();

		if (seconds <= 0)
		{
			explosher.setFingerState(Explosher.FingerState.STOP);
		}
		if (seconds >= 2 && seconds < 2.1)
		{
			explosher.setFingerState(Explosher.FingerState.PASS);
		}
		if (seconds > 0 && seconds < 7.2)
		{
			exploSwag(true, "Forward");
		}
		if (seconds > 4 && seconds < 4.1)
		{
			toggleY(true);
			DebugUtil.logAdd("first push on");
		}
		if (seconds > 4.1 && seconds < 4.2)
		{
			toggleY(false);
			DebugUtil.logAdd("first push off");
		}
		if (seconds > 5.5 && seconds < 6)
		{
			toggleY(true);
			DebugUtil.logAdd("second push on");
		}
		if (seconds >= 6 && seconds < 6.1)
		{
			toggleY(false);
			DebugUtil.logAdd("second push off");
		}
		if (seconds > 6 && seconds < 6.6)
		{
			toggleX(true);
		}
		if (seconds >= 6.6 && seconds < 6.7)
		{
			toggleX(false);
		}
		if (seconds > 6.6 && seconds < 7)
		{
			toggleY(true);
		}
		if (seconds >= 7.8)
		{
			toggleY(false);
			exploSwag(false, "Forward");
			shouldShoot = false;
			explosher.setFingerState(Explosher.FingerState.STOP);
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
		vaccum.swagReverse(pow);
	}
}