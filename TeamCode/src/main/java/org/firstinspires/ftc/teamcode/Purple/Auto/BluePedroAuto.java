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
import org.firstinspires.ftc.teamcode.Purple.Controls;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class BluePedroAuto extends OpMode {
    public Follower follower;
    public Explosher explosher;
    public Vaccum vaccum;

    private Controls driver1;
    private Controls driver2;

    private long lastTagSeenTime = 0;
    private double regressionSlope;
    private double regressionIntercept;
    public double dist;
    public boolean IsBusy;
    private static final double RPM_SMOOTHING_ALPHA = 0.2;
    private double smoothedTargetRPM = 0;

    private Timer pathTimer, opModeTimer, enumTimer;
    public enum PathState {
        // path from start to shoot posi
        START_SHOOT,
        // shoots preloads
        SHOOTPRELOAD,
        //
        FIRSTHALFLIFE1,

        FIRSTHALFLIFE2,

        SHOOT_LINE1,

        SECONDHALF_LIFE1,

        SECONDHALF_LIFE2,

        SHOOT_LINE2,

        RANKMOVE,

        DONE;


        public PathState next () {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.28301886792453, 123.84905660377358, Math.toRadians(143));
    private final Pose shootPose = new Pose(53.43396226415094, 94.41509433962264, Math.toRadians(143));

    private final Pose Pickup_First_Halflife1Pose = new Pose(53.440993788819874, 83.0323509898277, Math.toRadians(180));

    private final Pose Pickup_First_Halflife2Pose = new Pose(25, 83.0323509898277, Math.toRadians(180));

    private final Pose Pickup_Second_Halflife1Pose = new Pose(48.014815154531284, 57.12668327854573, Math.toRadians(180));

    private final Pose Pickup_Second_Halflife2Pose = new Pose(25, 59.13660577338656, Math.toRadians(180));

    private final Pose rankPose = new Pose(39.751800453518925, 62.93312604141928, Math.toRadians(90));

    private PathChain DriveStartShoot;

    private PathChain DriveToHalfLife1;

    private PathChain DriveHalfLife1;

    private PathChain DrivePickupShoot1;

    private PathChain DriveToHalfLife2;
    private PathChain DriveHalfLife2;

    private PathChain DrivePickupShoot2;
    private PathChain RankMove;

    public void buildpaths() {
        DriveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        DriveToHalfLife1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, Pickup_First_Halflife1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Pickup_First_Halflife1Pose.getHeading())
                .build();
        DriveHalfLife1 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup_First_Halflife1Pose, Pickup_First_Halflife2Pose))
                .setLinearHeadingInterpolation(Pickup_First_Halflife1Pose.getHeading(), Pickup_First_Halflife2Pose.getHeading())
                .build();
        DrivePickupShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup_First_Halflife2Pose, shootPose))
                .setLinearHeadingInterpolation(Pickup_First_Halflife2Pose.getHeading(), shootPose.getHeading())
                .build();
        DriveToHalfLife2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, Pickup_Second_Halflife1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Pickup_Second_Halflife1Pose.getHeading())
                .build();
        DriveHalfLife2 = follower.pathBuilder()
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

    private void shootState(PathState Pathstate) {
        shootFull();
        DebugUtil.logAdd("Finished shooting");
        if (pathTimer.getElapsedTimeSeconds() > 10) {
            setPathState(Pathstate);
        }
    }

    private void moveState(PathChain Pathchain, boolean holdEnd) {
        follower.followPath(Pathchain, holdEnd);
    }



    public void statePathUpdate() {
        switch (pathState) {
            case START_SHOOT:
                moveState(DriveStartShoot, true);
                setPathState(PathState.SHOOTPRELOAD);
                break;

            case SHOOTPRELOAD:
                if (!follower.isBusy()) {
                    shootState(PathState.FIRSTHALFLIFE1);
                }
                break;

            case FIRSTHALFLIFE1:
                if (!follower.isBusy()) {
                    moveState(DriveToHalfLife1, true);
                    setPathState(PathState.FIRSTHALFLIFE2);
                }
                break;

            case FIRSTHALFLIFE2:
                if (!follower.isBusy()) {
                    explosher.setFingerState(Explosher.FingerState.STOP);
                    vaccum.setPower(1.0);
                    exploSwag(false, "Forward");
                    moveState(DriveHalfLife1, true);
                    setPathState(PathState.SHOOT_LINE1);
                }

            case SHOOT_LINE1:
                if (!follower.isBusy()) {
                    vaccum.setPower(0);
                    moveState(DrivePickupShoot1, false);
                    if (enumTimer.getElapsedTimeSeconds() == 3.0) {
                        shootState(PathState.SECONDHALF_LIFE1);
                    }
                }
                break;

            case SECONDHALF_LIFE1:
                if (!follower.isBusy()) {
                    moveState(DriveToHalfLife2, true);
                    setPathState(PathState.SECONDHALF_LIFE2);
                }
                break;

            case SECONDHALF_LIFE2:
                if (!follower.isBusy()) {
                    vaccum.setPower(1.0);
                    exploSwag(false, "Forward");
                    moveState(DriveHalfLife2, false);
                    setPathState(PathState.SHOOT_LINE2);
                }

            case SHOOT_LINE2:
                if (!follower.isBusy()) {
                    vaccum.setPower(0.0);
                    moveState(DrivePickupShoot2, false);
                    if (enumTimer.getElapsedTimeSeconds() == 1) {
                        shootState(PathState.RANKMOVE);
                    }
                }
                break;

            case RANKMOVE:
                if (!follower.isBusy()) {
                    moveState(RankMove, true);
                }
                break;
            default:
                DebugUtil.logAdd("no current state");
                break;
        }
    }


    public void switchState() {
        if (driver1.isPressed("a")) {
            pathState.next();
        }
    }


    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        enumTimer.resetTimer();
    }

    @Override
    public void init() {
        driver1 = new Controls(gamepad1);
        pathState = PathState.START_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        enumTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildpaths();
        follower.setPose(startPose);

        if (driver1.isPressed("b")) {
            DebugUtil.logAdd("current pathstate: " + pathState);
        }
        if (follower.isBusy()) {
            DebugUtil.logAdd("busy");
        }
        DebugUtil.logAdd("position" + follower.getPose());


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
        if (pathTimer.getElapsedTimeSeconds() < 0 ) {
            explosher.setFingerState(Explosher.FingerState.STOP);
        }
        if (pathTimer.getElapsedTimeSeconds() > 2) {
            explosher.setFingerState(Explosher.FingerState.PASS);
        }
        if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 7.2) {
            exploSwag(true, "Forward");
        }
        if (pathTimer.getElapsedTimeSeconds() > 3 && pathTimer.getElapsedTimeSeconds() < 3.1) {
            toggleY(true);
            DebugUtil.logAdd("first push on");
        }
        if (pathTimer.getElapsedTimeSeconds() > 3.1 && pathTimer.getElapsedTimeSeconds() < 3.2) {
            toggleY(false);
            DebugUtil.logAdd("first push off");
        }
        if (pathTimer.getElapsedTimeSeconds() > 4.5 && pathTimer.getElapsedTimeSeconds() < 5) {
            toggleY(true);
            DebugUtil.logAdd("first push on");
        }
        if (pathTimer.getElapsedTimeSeconds() == 5) {
            toggleY(false);
            DebugUtil.logAdd("first push off");
        }
        if (pathTimer.getElapsedTimeSeconds() > 5 && pathTimer.getElapsedTimeSeconds() < 5.6) {
            toggleX(true);
        }
        if (pathTimer.getElapsedTimeSeconds() == 5.6) {
            toggleX(false);
        }
        if (pathTimer.getElapsedTimeSeconds() > 5.6 && pathTimer.getElapsedTimeSeconds() < 6) {
            toggleY(true);
        }
        if (pathTimer.getElapsedTimeSeconds() == 7.1) {
            toggleY(false);
            exploSwag(false, "Forward");
        }
    }

    @Override
    public void loop() {
        DebugUtil.update();
        LimeUtil.update();
        driver1.update();

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