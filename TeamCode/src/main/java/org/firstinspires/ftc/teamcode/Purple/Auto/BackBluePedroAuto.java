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

@Autonomous(name = "BackBluePedroAuto", group = "Purple")
public class BackBluePedroAuto extends OpMode
{

	private static final double RPM_SMOOTHING_ALPHA = 0.2;
	// sample poses (adjust to your field/layout)
	private final Pose startPose = new Pose(62.5, 8, Math.toRadians(90));
	private final Pose shootPose = new Pose(62.5, 23, Math.toRadians(118));
	private final Pose Pickup_First_Halflife1Pose = new Pose(48, 35.5, Math.toRadians(180));
	private final Pose Pickup_First_Halflife2Pose = new Pose(25, 35.5, Math.toRadians(180));
	private final Pose Pickup_Second_Halflife1Pose = new Pose(48.014815154531284, 61, Math.toRadians(180));
	private final Pose Pickup_Second_Halflife2Pose = new Pose(25, 60, Math.toRadians(180));
	private final Pose rankPose = new Pose(39.751800453518925, 62.93312604141928, Math.toRadians(90));
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

	private void onCreate ()
	{
		// create follower and purple path manager
		follower = Constants.createFollower(hardwareMap);
		follower.setPose(startPose);
		pathManager = new PurplePathing(follower);

		shootTimer = new ElapsedTime();

		calculateRegression();

		LimeUtil.start(hardwareMap, 60);
		LimeUtil.setPipeline(0);
		explosher = new Explosher(hardwareMap);
		vaccum = new Vaccum(hardwareMap);

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
		explosher.setGateState(Explosher.GateState.STOP);
	}

	private void shootFull ()
	{
		// Start Stuff

		if (shootTimer.seconds() <= 0)
		{
			explosher.setGateState(Explosher.GateState.STOP);
		}
		if (shootTimer.seconds() > 2)
		{
			explosher.setGateState(Explosher.GateState.PASS);
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
			explosher.setGateState(Explosher.GateState.STOP);
		}
	}

	public void exploSwag (boolean should, String Explostate)
	{

		if (should)
		{
			if (Explostate == "Forward")
			{
				if (LimeUtil.getTd() != 0)
				{
					dist = LimeUtil.getTd();
					double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;
					smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
					smoothedTargetRPM = Math.max(0, Math.min(smoothedTargetRPM, explosher.getMaxRPM()));
					explosher.setRPM(smoothedTargetRPM);
				}
			}
			else if (Explostate == "Back")
			{
				explosher.setRPM(-4000);
			}
		}
		else
		{
			explosher.stop();
		}
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
}
