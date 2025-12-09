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
        //
        PICKUP1,

        SHOOT_LINE1,

        HALF_LIFE2,

        SHOOT_LINE2,

        RANKMOVE,

        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(122.31055900621118, 124.77018633540374, Math.toRadians(37));
    private final Pose shootPose = new Pose(83.85093167701864, 83.40372670807454, Math.toRadians(44));

    private final Pose PickupPose1 = new Pose(121.6, 83.6273291925466, Math.toRadians(0));

    private final Pose Pickup_Second_Halflife1Pose = new Pose(96.4, 59.3, Math.toRadians(0));

    private final Pose Pickup_Second_Halflife2Pose = new Pose(121.6, 59.254658385093165, Math.toRadians(0));

    private final Pose rankPose = new Pose(105.98757763975155, 72.67080745341613, Math.toRadians(90));

    private PathChain DriveStartShoot;

    private PathChain DriveShootPickup1;

    private PathChain DrivePickupShoot1;

    private PathChain DriveShootPickup2;

    private PathChain DriveToHalfLife;
    private PathChain DriveHalfLife;

    private PathChain DrivePickupShoot2;
    private PathChain RankMove;

    public void buildpaths() {
        DriveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        DriveShootPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, PickupPose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), PickupPose1.getHeading())
                .build();
        DrivePickupShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(PickupPose1, shootPose))
                .setLinearHeadingInterpolation(PickupPose1.getHeading(), shootPose.getHeading())
                .build();
        DriveToHalfLife = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, Pickup_Second_Halflife1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Pickup_Second_Halflife1Pose.getHeading())
                .build();
        DriveHalfLife = follower.pathBuilder()
                .addPath(new BezierLine(Pickup_Second_Halflife1Pose, Pickup_Second_Halflife2Pose))
                .setLinearHeadingInterpolation(Pickup_Second_Halflife1Pose.getHeading(), Pickup_Second_Halflife2Pose.getHeading())
                .build();
        DrivePickupShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup_Second_Halflife2Pose, shootPose))
                .setLinearHeadingInterpolation(Pickup_Second_Halflife2Pose.getHeading(), shootPose.getHeading())
                .build();
        RankMove = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rankPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), rankPose.getHeading())
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
                    if (pathTimer.getElapsedTimeSeconds() > 10) {
                        setPathState(PathState.PICKUP1);
                    }
                }
                break;

            case PICKUP1:
                if (!follower.isBusy()) {
                    vaccum.setPower(1.0);
                    follower.followPath(DriveShootPickup1, true);
                    setPathState(PathState.SHOOT_LINE1);
                }
                break;

            case SHOOT_LINE1:
                if (!follower.isBusy()) {
                    vaccum.setPower(0);
                    follower.followPath(DrivePickupShoot1, true);
                    shootFull();
                    if (pathTimer.getElapsedTimeSeconds() > 10) {
                        setPathState(PathState.HALF_LIFE2);
                    }
                }
                break;

            case HALF_LIFE2:
                if (!follower.isBusy()) {
                    follower.followPath(DriveToHalfLife, true);
                    vaccum.setPower(1.0);
                    follower.followPath(DriveHalfLife, true);
                    setPathState(PathState.SHOOT_LINE2);
                }
                break;

            case SHOOT_LINE2:
                if (!follower.isBusy()) {
                    vaccum.setPower(0.0);
                    follower.followPath(DrivePickupShoot2, true);
                    if (pathTimer.getElapsedTimeSeconds() > 10) {
                        shootFull();
                    }
                    setPathState(PathState.RANKMOVE);
                }
                break;

            case RANKMOVE:
                if (!follower.isBusy()) {
                    follower.followPath(RankMove, false);
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
        if (pathTimer.getElapsedTimeSeconds() < .2) {
            toggleB(true);
        }
        if (pathTimer.getElapsedTimeSeconds() > .2 && pathTimer.getElapsedTimeSeconds()< .35) {
            toggleB(false);
        }
        if (pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 8) {
            exploSwag(true, "Forward");
        }
        if (pathTimer.getElapsedTimeSeconds() > 3 && pathTimer.getElapsedTimeSeconds() < 3.1) {
            toggleY(true);
        }
        if (pathTimer.getElapsedTimeSeconds() > 3.1 && pathTimer.getElapsedTimeSeconds() < 3.3) {
            toggleY(false);
        }
        if (pathTimer.getElapsedTimeSeconds() > 4.5 && pathTimer.getElapsedTimeSeconds() < 4.7) {
            toggleY(true);
        }
        if (pathTimer.getElapsedTimeSeconds() > 4.7 && pathTimer.getElapsedTimeSeconds() < 4.8) {
            toggleY(false);
        }
        if (pathTimer.getElapsedTimeSeconds() > 5 && pathTimer.getElapsedTimeSeconds() < 5.05) {
            toggleX(true);
        }
        if (pathTimer.getElapsedTimeSeconds() > 5.05  && pathTimer.getElapsedTimeSeconds() < 5.05) {
            toggleX(false);
        }
        if (pathTimer.getElapsedTimeSeconds() > 5.5 && pathTimer.getElapsedTimeSeconds() < 6) {
            toggleY(true);
        }
        if (pathTimer.getElapsedTimeSeconds() > 7) {
            toggleY(false);
        }
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
        double pow = should ? -.15  : 0;
        vaccum.setPower(pow);
    }

    private void toggleSuperX(boolean should)
    {
        double rpm = should ? -4000 : 0.0;
        double pow = should ? -Vaccum.DEFAULT_POW : 0;
        exploSwag(should, should ? "Back" : "Off");
        vaccum.setPower(pow);
    }

    private void toggleB(boolean should)
    {
        double rpm = should ? -4000 : 0.0;
        double pow = should ? -.50 : 0;
        exploSwag(should, should ? "Back" : "Off");
        vaccum.swagReverse(pow);
    }
}