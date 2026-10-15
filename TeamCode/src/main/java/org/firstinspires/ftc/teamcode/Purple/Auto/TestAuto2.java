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
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurpleChain;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePath;
import org.firstinspires.ftc.teamcode.Purple.Pathing.PurplePathing;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import java.util.ArrayList;
import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TestAuto2", group = "Purple")

public class TestAuto2 extends PurpleOpMode {

    public static PathConstraints defaultConstraints = new PathConstraints(0.995, 0.1, 0.1, 0.007, 100, .5, 10, .5);

    private final Pose startPose = new Pose(15, 84, Math.toRadians(90));
    private final Pose shootPose = new Pose(100, 84, Math.toRadians(90));

    private final Pose finPose = new Pose(100, 48, Math.toRadians(90));

    public double dist;
    public ElapsedTime timer;
    private Follower follower;

    private PurplePathing pathManager;


    @Override
    public void create() {

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        follower.setConstraints(defaultConstraints);
        pathManager = new PurplePathing(follower);
        timer = new ElapsedTime();

        PurpleMemory.initialize(hardwareMap, follower);
        LimeUtil.start(hardwareMap, 60);
        LimeUtil.setPipeline(0);

        // Path Chain Presets
        PathChain DriveStartPickup = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        PathChain DriveShoot = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, finPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finPose.getHeading())
                .build();
//        PathChain Drive = follower.pathBuilder()
//                .addPath(new BezierLine(finPose, startPose))
//                .setLinearHeadingInterpolation(finPose.getHeading(), startPose.getHeading())
//                .build();

        // Create the PurplePath objects
        PurplePath path1 = new PurplePath("drive1", DriveStartPickup, 9.0, 5);

        PurplePath path2 = new PurplePath("drive2", DriveShoot, 1.0, 0)
                .onComplete(() -> DebugUtil.logAdd("path2 completed"));
//
//        PurplePath path3 = new PurplePath("drive3", DriveShoot, 1, 1)
//                .onComplete(() -> DebugUtil.logAdd("path3 completed"));

        // Create the PurpleChain object
        PurpleChain chain = new PurpleChain(path1, path2)
                .onComplete(() -> DebugUtil.logAdd("Chain fully finished"));

        // Start the chain. holdEnd = true (follower will hold at end of each path)
        pathManager.startChain(chain, false, () -> DebugUtil.logAdd("startChain() provided onComplete"));

        DebugUtil.setTelemetry(telemetry);
    }

    @Override
    public void update() {

        follower.update();
        LimeUtil.update();
        PurpleMemory.Instance.update();
        pathManager.update();


        DebugUtil.logAdd("Current Path: " + pathManager.curPath());
        DebugUtil.update();
    }

    @Override
    public void destroy (){}


}