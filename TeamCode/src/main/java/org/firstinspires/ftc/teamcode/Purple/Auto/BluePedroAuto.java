package org.firstinspires.ftc.teamcode.Purple.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class BluePedroAuto extends OpMode {
    public Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {
        // path from start to shoot posi
        START_SHOOT,
        // shoots preloads
        SHOOTPRELOAD,
        // move out of triangle
        OUT_OF_TRIANGLE
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.28301886792453, 123.84905660377358, Math.toRadians(143));
    private final Pose shootPose = new Pose(53.43396226415094, 94.41509433962264, Math.toRadians(143));

    private PathChain DriveStartShoot;

    public void buildpaths() {
        DriveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START_SHOOT:
                follower.followPath(DriveStartShoot, true);
                setPathState(pathState.SHOOTPRELOAD);
                break;
            case SHOOTPRELOAD:
                if (!follower.isBusy()) {
                    follower.followPath(DriveStartShoot, true);
                    DebugUtil.logAdd("Finished shooting");
                }
                break;
            default:
                DebugUtil.logAdd("no current state");
                break;
        }
    }


    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }



    @Override
    public void init() {
        pathState = PathState.START_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildpaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        DebugUtil.logAdd("pathstate" + pathState.toString());
        DebugUtil.logAdd("x: " + follower.getPose().getX());
        DebugUtil.logAdd("y: " + follower.getPose().getY());
        DebugUtil.logAdd("heading: " + follower.getHeading());
        DebugUtil.logAdd("path time: " + pathTimer.getElapsedTimeSeconds());
    }
}
