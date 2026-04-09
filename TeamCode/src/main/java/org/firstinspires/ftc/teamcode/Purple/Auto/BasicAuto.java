package org.firstinspires.ftc.teamcode.Purple.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

import java.util.ArrayList;
import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BasicBlueAuto", group = "Purple")
public class BasicAuto extends PurpleOpMode
{
	private static final double EXPLOSHER_DEFAULT_RPM = 2800.0;
	private static final double EXPLOSHER_COAST_ALPHA = 0.08;
	private static final double SHOT_SPINUP_SECONDS = 0.75;
	private static final double SHOT_FEED_SECONDS = 1.10;
	private static final double SHOT_FINISH_SECONDS = 1.40;

	public static PathConstraints defaultConstraints = new PathConstraints(0.995, 0.1, 0.1, 0.007, 100, .1, 10, .4);

	private final Pose startPose = new Pose(21, 127, Math.toRadians(235));
	private final Pose shootPose = new Pose(54, 80, Math.toRadians(180));
	private final Pose between = new Pose(57, 83, Math.toRadians(180));
	private final Pose pickup1 = new Pose(26, 83, Math.toRadians(180));
	private final Pose pickup2 = new Pose(27, 59, Math.toRadians(180));
	private final Pose grabCurve = new Pose(50, 52);
	private final Pose firstCurve = new Pose(68, 78);
	private final Pose rankPose = new Pose(53, 60, Math.toRadians(150));
	private final ArrayList<Pose> pick2 = new ArrayList<>(Arrays.asList(shootPose, grabCurve, pickup2));
	private final ArrayList<Pose> pick1 = new ArrayList<>(Arrays.asList(startPose, firstCurve, pickup1));

	public ElapsedTime shootTimer;

	private Follower follower;
	private Explosher explosher;
	private Vaccum vaccum;
	private double desiredExplosherRPM = EXPLOSHER_DEFAULT_RPM;
	private double rememberedRegressedRPM = EXPLOSHER_DEFAULT_RPM;
	private PurplePathing pathManager;
	private boolean shotActive = false;

	@Override
	public void create ()
	{
		follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
		follower.setPose(startPose);
		pathManager = new PurplePathing(follower);

		PurpleMemory.initialize(hardwareMap, follower);
		LimeUtil.start(hardwareMap, 60);
		LimeUtil.setPipeline(0);

		shootTimer = new ElapsedTime();

		explosher = new Explosher(hardwareMap);
		explosher.setAimPoint(72, 144);
		explosher.setRegressionEnabled(true);

		vaccum = new Vaccum(hardwareMap);

		PathChain driveIdle = follower.pathBuilder()
				.addPath(new BezierLine(startPose, startPose))
				.setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
				.build();
		PathChain driveStartPickup = follower.pathBuilder()
				.addPath(new BezierLine(startPose, between))
				.setLinearHeadingInterpolation(startPose.getHeading(), between.getHeading())
				.build();
		PathChain driveBetween = follower.pathBuilder()
				.addPath(new BezierLine(between, pickup1))
				.setLinearHeadingInterpolation(between.getHeading(), pickup1.getHeading())
				.build();
		PathChain driveOpen1 = follower.pathBuilder()
				.addPath(new BezierLine(pickup1, shootPose))
				.setLinearHeadingInterpolation(pickup1.getHeading(), shootPose.getHeading())
				.build();
		PathChain driveShootPickup = follower.pathBuilder()
				.addPath(new BezierCurve(pick2, defaultConstraints))
				.setLinearHeadingInterpolation(shootPose.getHeading(), pickup2.getHeading())
				.build();
		PathChain driveOpen2 = follower.pathBuilder()
				.addPath(new BezierLine(pickup2, shootPose))
				.setLinearHeadingInterpolation(pickup2.getHeading(), shootPose.getHeading())
				.build();
		PathChain rankMove = follower.pathBuilder()
				.addPath(new BezierLine(shootPose, rankPose))
				.setLinearHeadingInterpolation(shootPose.getHeading(), rankPose.getHeading())
				.build();

		PurplePath path1 = new PurplePath("Idle", driveIdle, 0.0, 0.0)
				.onComplete(shootTimer::reset);

		PurplePath path2 = new PurplePath("first pickup", driveStartPickup, 5.0, 5.0)
				.onComplete(this::beginMatch);

		PurplePath path3 = new PurplePath("second pickup", driveBetween, 3.0, 5.0)
				.onComplete(vaccum::stop);

		PurplePath path4 = new PurplePath("return to shoot", driveOpen1, 3.0, 5.0)
				.onComplete(() -> vaccum.setPower(0.5));

		PurplePath path5 = new PurplePath("pick the up", driveShootPickup, 3.0, 5.0)
				.onComplete(vaccum::stop);

		PurplePath path6 = new PurplePath("shoot", driveOpen2, 3.0, 5.0)
				.onComplete(this::beginTimedShot);

		PurplePath path7 = new PurplePath("rank", rankMove, 3.0, 5.0)
				.onComplete(() -> DebugUtil.logAdd("path2 completed"));

		PurpleChain chain = new PurpleChain(path1, path2, path3, path4, path5, path6, path7)
				.onComplete(() -> DebugUtil.logAdd("Chain fully finished"));

		pathManager.startChain(chain, true, () -> DebugUtil.logAdd("startChain() provided onComplete"));

		shootTimer.reset();
		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update ()
	{
		follower.update();
		LimeUtil.update();
		PurpleMemory.Instance.update();
		pathManager.update();

		explosher.update();
		vaccum.update();

		updateExplosher();
		updateShotSequence();
		teleInfo();

		DebugUtil.logAdd("Current Path: " + pathManager.curPath());
		DebugUtil.update();
	}

	@Override
	public void destroy ()
	{
		explosher.stop();
		vaccum.stop();
	}

	private void updateExplosher ()
	{
		explosher.setRegressionEnabled(true);

		if (explosher.hasRegressionTarget())
		{
			rememberedRegressedRPM = explosher.getSmoothedTargetRPM();
			desiredExplosherRPM = rememberedRegressedRPM;
		}
		else
		{
			desiredExplosherRPM += EXPLOSHER_COAST_ALPHA * (EXPLOSHER_DEFAULT_RPM - desiredExplosherRPM);
		}

		desiredExplosherRPM = Math.max(0.0, Math.min(desiredExplosherRPM, explosher.getMaxRPM()));
		explosher.setRPM(desiredExplosherRPM);
	}

	private void beginMatch ()
	{
		explosher.setAimPoint(15, 130);
		vaccum.setPower(0.5);
	}

	private void beginTimedShot ()
	{
		shotActive = true;
		shootTimer.reset();
		explosher.setGateState(Explosher.GateState.STOP);
		vaccum.stop();
	}

	private void updateShotSequence ()
	{
		if (!shotActive)
		{
			return;
		}

		double elapsedSeconds = shootTimer.seconds();
		if (elapsedSeconds < SHOT_SPINUP_SECONDS)
		{
			explosher.setGateState(Explosher.GateState.STOP);
			vaccum.stop();
			return;
		}

		if (elapsedSeconds < SHOT_FEED_SECONDS)
		{
			explosher.setGateState(Explosher.GateState.PASS);
			vaccum.setPower(0.65);
			return;
		}

		if (elapsedSeconds < SHOT_FINISH_SECONDS)
		{
			explosher.setGateState(Explosher.GateState.PASS);
			vaccum.stop();
			return;
		}

		explosher.setGateState(Explosher.GateState.STOP);
		vaccum.stop();
		shotActive = false;
	}

	private void teleInfo ()
	{
		DebugUtil.logAdd("============== [LIME]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("POSE: " + LimeUtil.getResult().getBotpose());
		DebugUtil.logAdd("Target X: " + LimeUtil.getTx());
		DebugUtil.logAdd("Target D: " + LimeUtil.getTd());
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("============== [EXPLOSHER]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Target RPM: " + explosher.getTargetRPM());
		DebugUtil.logAdd("Current RPM: " + explosher.getCurrentRPM());
		DebugUtil.logAdd("Smoothed Regress: " + explosher.getSmoothedTargetRPM());
		Double odoDistInches = explosher.getDistanceToTarget();
		DebugUtil.logAdd("[ODOMETRY] Target Dist: " + (odoDistInches == null ? "N/A" : odoDistInches));
		DebugUtil.logAdd("Gate State: " + explosher.getGateState());
		DebugUtil.logAdd("Gate Position: " + String.format("%.3f", explosher.getGatePosition()));
		DebugUtil.logAdd("Shot Active: " + shotActive);
		DebugUtil.logAdd(" ");

		Pose followerPose = follower.getPose();
		DebugUtil.logAdd("[POSITION] HEADING: " + Math.toDegrees(followerPose.getHeading()));
		DebugUtil.logAdd("[POSITION] X: " + PurpleMemory.Instance.curPos().getX());
		DebugUtil.logAdd("[POSITION] Y: " + PurpleMemory.Instance.curPos().getY());
		DebugUtil.logAdd("Intake Power: " + String.format("%.2f", vaccum.getPower()));
		DebugUtil.logAdd(" ");
	}
}
