package org.firstinspires.ftc.teamcode.Purple.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class BluePedroAuto extends OpMode {
    public Follower follower;
    public Explosher explosher;
    public Vaccum vaccum;

    private long lastTagSeenTime = 0;
    private double regressionSlope;
    private double regressionIntercept;
    public double dist;
    private static final double RPM_SMOOTHING_ALPHA = 0.2;
    private double smoothedTargetRPM = 0;

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
                setPathState(PathState.SHOOTPRELOAD);
                break;
            case SHOOTPRELOAD:
                if (!follower.isBusy()) {
                    shootFull();
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

        DebugUtil.setTelemetry(telemetry);

        calculateRegression();

        initializeVaccum();
        initializeExplosher();

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    private void shootFull() {
        if (pathTimer.getElapsedTimeSeconds() < 1) {
            toggleB(true);
        }
//        if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 8) {
//            exploSwag(true, "Forward");
//        }
//        if (pathTimer.getElapsedTimeSeconds() > 2.3 && pathTimer.getElapsedTimeSeconds() < 2.5) {
//            toggleY(true);
//        }
//        if (pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 2.6) {
//            toggleY(false);
//        }
//        if (pathTimer.getElapsedTimeSeconds() > 4 && pathTimer.getElapsedTimeSeconds() < 4.3) {
//            toggleY(true);
//        }
//        if (pathTimer.getElapsedTimeSeconds() > 4.3 && pathTimer.getElapsedTimeSeconds() < 4.4) {
//            toggleY(false);
//        }
//        if (pathTimer.getElapsedTimeSeconds() > 5 && pathTimer.getElapsedTimeSeconds() < 5.1) {
//            toggleX(true);
//        }
//        if (pathTimer.getElapsedTimeSeconds() > 5.2 && pathTimer.getElapsedTimeSeconds() < 5.4) {
//            toggleY(true);
//        }
//        if (pathTimer.getElapsedTimeSeconds() > 10) {
//            toggleY(false);
//        }
    }

    @Override
    public void loop() {
        DebugUtil.update();
        LimeUtil.update();

        follower.update();

        statePathUpdate();

        DebugUtil.logAdd("pathstate: " + pathState.toString());
        DebugUtil.logAdd("x: " + follower.getPose().getX());
        DebugUtil.logAdd("y: " + follower.getPose().getY());
        DebugUtil.logAdd("heading: " + follower.getHeading());
        DebugUtil.logAdd("path time: " + pathTimer.getElapsedTimeSeconds());
        if (LimeUtil.getTargetDistance() != 0) {
            DebugUtil.logAdd("apriltag distance: " + LimeUtil.getTargetDistance());
            DebugUtil.logAdd("smooth rpm: " + smoothedTargetRPM);
        } else {
            DebugUtil.logAdd("no apriltag found");
        }

        explosher.update();
        vaccum.update();
    }



    public void exploSwag(boolean should, String Explostate)
    {
        if (should)
        {
            if (Explostate == "Forward")
            {
                if (LimeUtil.getTargetDistance() != 0) {
                    dist = LimeUtil.getTargetDistance();
                    double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;
                    smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
                    smoothedTargetRPM = Math.max(0, Math.min(smoothedTargetRPM, explosher.getMaxRPM()));
                    explosher.setRPM(smoothedTargetRPM);
                }
            }
            else if (Explostate == "Back") {
                explosher.setRPM(-4000);
            }
        }
        else {
            explosher.stop();
        }
    }

    private void initializeExplosher ()
    {
        LimeUtil.start(hardwareMap, "SwagLime", 60);
        LimeUtil.setPipeline(0);
        explosher = new Explosher(hardwareMap, org.firstinspires.ftc.teamcode.Purple.Constants.FINGER_SERVO_CONFIG);
    }
    private void initializeVaccum ()
    {
        vaccum = new Vaccum(hardwareMap);
    }

    private void calculateRegression ()
    {

        double[][] calibrationPoints = {
                {59, 3000},
                {65, 2800},
                {77, 3100},
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

    private void toggleY(boolean should)
    {
        double pow = should ? Vaccum.DEFAULT_POW : 0;
        vaccum.setPower(pow);
    }

    private void toggleX(boolean should)
    {
        double pow = should ? -Vaccum.DEFAULT_POW : 0;
        vaccum.setPower(pow);
    }

    private void toggleSuperX(boolean should)
    {
        double rpm = should ? -4000 : 0.0;
        double pow = should ? -Vaccum.DEFAULT_POW : 0;
        explosher.setRPM(rpm);
        vaccum.setPower(pow);
    }

    private void toggleB(boolean should)
    {
        double rpm = should ? -4000 : 0.0;
        double pow = should ? -Vaccum.DEFAULT_POW : 0;
        explosher.setRPM(rpm);
        vaccum.swagReverse(pow);
    }

}