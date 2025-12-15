package org.firstinspires.ftc.teamcode.Purple.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "SwagAuto", group = "Purple")
public class SwagAuto extends OpMode
{

	// sample poses (adjust to your field/layout)
	private final Pose startPose = new Pose(21.28301886792453, 123.84905660377358, Math.toRadians(143));
	private final Pose backPose = new Pose(21.28301886792453, 110.0, Math.toRadians(143)); // move back ~14 in
	private final Pose rightPose = new Pose(26.28301886792453, 110.0, Math.toRadians(143)); // move right ~5 in
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

		// Build Pedro PathChains for the two movements
		PathChain driveBack = follower.pathBuilder()
				.addPath(new BezierLine(startPose, backPose))
				.setLinearHeadingInterpolation(startPose.getHeading(), backPose.getHeading())
				.build();

		PathChain driveRight = follower.pathBuilder()
				.addPath(new BezierLine(backPose, rightPose))
				.setLinearHeadingInterpolation(backPose.getHeading(), rightPose.getHeading())
				.build();

		PurplePath path1 = new PurplePath("Drive Back", driveBack, 2.0, 3.0)
				.onComplete(() -> DebugUtil.logAdd("path1 completed"));

		PurplePath path2 = new PurplePath("Drive Right Slight", driveRight, 1.5, 0.0)
				.onComplete(() -> DebugUtil.logAdd("path2 completed"));

		// Create a chain: path1 then path2. Provide a chain-level onComplete too.
		PurpleChain chain = new PurpleChain(path1, path2)
				.onComplete(() -> DebugUtil.logAdd("Chain fully finished"));

		// Start the chain. holdEnd = true (follower will hold at end of each path)
		pathManager.startChain(chain, true, () -> DebugUtil.logAdd("startChain() provided onComplete"));

		DebugUtil.setTelemetry(telemetry);
	}

	private void onUpdate ()
	{
		// update the manager (must be called every loop)
		pathManager.update();

		// logging
		DebugUtil.logAdd("Current Path: " + pathManager.curPath());
		DebugUtil.update();
	}
}
