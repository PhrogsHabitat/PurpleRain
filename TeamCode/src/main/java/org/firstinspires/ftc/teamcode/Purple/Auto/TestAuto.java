package org.firstinspires.ftc.teamcode.Purple.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TestAuto", group = "Purple")
public class TestAuto extends PurpleOpMode
{
	private static final double EXPLOSHER_DEFAULT_RPM = 2800.0;
	private static final double EXPLOSHER_COAST_ALPHA = 0.08;
	private static final Pose DEFAULT_AUTO_START_POSE = new Pose(72, 72, Math.toRadians(0.0));
	public static PathConstraints defaultConstraints = new PathConstraints(0.995, 0.1, 0.1, 0.007, 100, 1, 10, 1);
	private final Pose startPose = new Pose(87, 8.5, Math.toRadians(90));
	private final Pose rankPose = new Pose(87, 43, Math.toRadians(90));
	// make the lists
	public ElapsedTime shootTimer;
	public double dist;
	private Follower follower;
	private Explosher explosher;
	private Vaccum vaccum;
	private double desiredExplosherRPM = EXPLOSHER_DEFAULT_RPM;

//    private final ArrayList<Pose> Pick2 = new ArrayList<>(Arrays.asList(shootPose, GrabCurve, Pickup2));
	private double rememberedRegressedRPM = EXPLOSHER_DEFAULT_RPM;
	private double rememberedRegressedHood = Constants.FINGER_STOP_POSITION;
	private Explosher.FingerState fingerState = Explosher.FingerState.STOP;
	private PurplePathing pathManager;

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
//        explosher.setTarget(132, 135);
		explosher.setRegressionEnabled(true);

		vaccum = new Vaccum(hardwareMap);

		// Path Chain Presets
		PathChain DriveStartPickup = follower.pathBuilder()
				.addPath(new BezierLine(startPose, rankPose))
				.setLinearHeadingInterpolation(startPose.getHeading(), rankPose.getHeading())
				.build();

		// Create the PurplePath objects
		PurplePath path1 = new PurplePath("first pickup", DriveStartPickup, 5.0, 2)
				.onComplete(() -> DebugUtil.logAdd("path2 completed"));

		// Create the PurpleChain object
		PurpleChain chain = new PurpleChain(path1)
				.onComplete(() -> DebugUtil.logAdd("Chain fully finished"));

		// Start the chain. holdEnd = true (follower will hold at end of each path)
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

		updateDrive();
		updateExplosher();
		updateVaccum();
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

	private void updateDrive ()
	{

		// AutoBase has no manual drive controls.
		follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
	}

	private void updateExplosher ()
	{

		explosher.setRegressionEnabled(true);

		if (explosher.hasRegressionTarget())
		{
			rememberedRegressedRPM = explosher.getSmoothedTargetRPM();
			rememberedRegressedHood = explosher.getSmoothedTargetHoodPosition();
			desiredExplosherRPM = rememberedRegressedRPM;
			if (explosher.getFingerStateEnum() != Explosher.FingerState.DEBUG)
			{
				explosher.setFingerPosition(rememberedRegressedHood);
			}
		}
		else
		{
			desiredExplosherRPM += EXPLOSHER_COAST_ALPHA * (EXPLOSHER_DEFAULT_RPM - desiredExplosherRPM);
		}

		desiredExplosherRPM = Math.max(0.0, Math.min(desiredExplosherRPM, explosher.getMaxRPM()));
		explosher.setRPM(desiredExplosherRPM);

		explosher.updateAim(PurpleMemory.Instance.curPos());
	}

	private void updateVaccum ()
	{

		// AutoBase has no manual vacuum controls.
		vaccum.stop();
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
		DebugUtil.logAdd("Smoothed Hood Regress: " + explosher.getSmoothedTargetHoodPosition());
		Double odoDistInches = explosher.getDistanceToTarget();
		DebugUtil.logAdd("[ODOMETRY] Target Dist: " + (odoDistInches == null ? "N/A" : odoDistInches));
		DebugUtil.logAdd("Auto Aim: ON");
		DebugUtil.logAdd("Exploring Pos: " + explosher.getExploringPos());
		DebugUtil.logAdd(String.format("Exploring PID: out=%.3f err=%.2f", explosher.getAimPow(), explosher.getAimErr()));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("============== [FINGER]");
		DebugUtil.logAdd(" ");
		DebugUtil.logAdd("Finger State: " + fingerState);
		DebugUtil.logAdd("Finger Position: " + String.format("%.3f", explosher.getFingerPosition()));
		DebugUtil.logAdd(String.format("Vacuum Finger 1 Pos: %.3f", vaccum.getFingerPosition(0)));
		DebugUtil.logAdd(String.format("Vacuum Finger 2 Pos: %.3f", vaccum.getFingerPosition(1)));
		DebugUtil.logAdd(String.format("Vacuum Finger 3 Pos: %.3f", vaccum.getFingerPosition(2)));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("============== [MEMORY]");
		DebugUtil.logAdd("[MOTIF]: " + PurpleMemory.Instance.curMotif());
		DebugUtil.logAdd("[BALLS]: " + Arrays.toString(PurpleMemory.Instance.curBalls()));
		DebugUtil.logAdd(" ");

		Pose followerPose = follower.getPose();
		DebugUtil.logAdd("[POSITION] HEADING: " + Math.toDegrees(followerPose.getHeading()));
		DebugUtil.logAdd("[POSITION] X: " + PurpleMemory.Instance.curPos().getX());
		DebugUtil.logAdd("[POSITION] Y: " + PurpleMemory.Instance.curPos().getY());
		DebugUtil.logAdd(" ");

		DebugUtil.update();
	}
}
