package org.firstinspires.ftc.teamcode.Purple.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
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

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "UpdatedBlueAuto", group = "Purple")
public class UpdatedBlueAuto extends OpMode
{

    private Follower follower;

    public Explosher explosher;
    public Vaccum vaccum;

    public ElapsedTime shootTimer;

    private double regressionSlope;

    private double regressionIntercept;

    public double dist;

    private static final double RPM_SMOOTHING_ALPHA = 0.2;

    private double smoothedTargetRPM = 0;

    private boolean shouldShoot = false;

    private boolean shootDone = true;

    // sample poses (adjust to your field/layout)
    public static PathConstraints defaultConstraints = new PathConstraints(0.995, 0.1, 0.1, 0.007, 100, .1, 10, .4);
    private final Pose startPose = new Pose(20, 124, Math.toRadians(144));
    private final Pose shootPose = new Pose(54, 84, Math.toRadians(130));
    private final Pose Pickup1 = new Pose(26, 83, Math.toRadians(180));
    private final Pose Open1 = new Pose(17, 75.5, Math.toRadians(180));
    private final Pose Pickup2 = new Pose(27, 59, Math.toRadians(180));
    private final Pose GrabCurve = new Pose(50, 52);
    private final Pose Open2 = new Pose(17, 66, Math.toRadians(180));
    private final Pose OpenGrab = new Pose(13, 60, Math.toRadians(150));
    private final Pose FirstCurve = new Pose(68, 78);
//    private final Pose OpenGrabCurve = new Pose(27.7, 51);
    private final Pose rankPose = new Pose(53, 60, Math.toRadians(150));
    private final ArrayList<Pose> Pick2 = new ArrayList<>(Arrays.asList(shootPose, GrabCurve, Pickup2));
//    private final ArrayList<Pose> loop = new ArrayList<>(Arrays.asList(shootPose, OpenGrabCurve, OpenGrab));
    private final ArrayList<Pose> Pick1 = new ArrayList<>(Arrays.asList(startPose, FirstCurve, Pickup1));
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

        LimeUtil.start(hardwareMap, "SwagLime", 60);
        LimeUtil.setPipeline(0);
        explosher = new Explosher(hardwareMap, org.firstinspires.ftc.teamcode.Purple.Constants.FINGER_SERVO_CONFIG);
        vaccum = new Vaccum(hardwareMap);


        // Path Chain Presets
        PathChain DriveStartPickup = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        PathChain DriveBetween = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, Pickup1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Pickup1.getHeading())
                .build();
        PathChain DriveOpen1 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup1, Open1))
                .setLinearHeadingInterpolation(Pickup1.getHeading(), Open1.getHeading())
                .build();
        PathChain DriveOpenShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Open1, shootPose))
                .setLinearHeadingInterpolation(Open1.getHeading(), shootPose.getHeading())
                .build();
        PathChain DriveShootPickup = follower.pathBuilder()
                .addPath(new BezierCurve(Pick2, defaultConstraints))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Pickup2.getHeading())
                .build();
        PathChain DriveOpen2 = follower.pathBuilder()
                .addPath(new BezierLine(Pickup2, Open2))
                .setLinearHeadingInterpolation(Pickup2.getHeading(), Open2.getHeading())
                .build();
        PathChain DriveOpenShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Open2, shootPose))
                .setLinearHeadingInterpolation(Open2.getHeading(), shootPose.getHeading())
                .build();
//        PathChain DriveOpenPickup = follower.pathBuilder()
//                .addPath(new BezierCurve(loop, defaultConstraints))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), OpenGrab.getHeading())
//                .build();
        PathChain RankMove = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rankPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), rankPose.getHeading())
                .build();

        // Create the PurplePath objects
        PurplePath path1 = new PurplePath("first pickup", DriveStartPickup, 5.0, 2.0)
                .onComplete(() -> flagShoot());

        PurplePath path2 = new PurplePath("second pickup", DriveBetween, 5.0, 2.0)
                .onComplete(() -> pickupBalls(true));

        PurplePath path3 = new PurplePath("OPEN THE GATE", DriveOpen1, 5.0, 2.0)
                .onComplete(() -> pickupBalls(false));

        PurplePath path4 = new PurplePath("shoot", DriveOpenShoot1, 5.0, 2.0)
                .onComplete(() -> doBoth(true));

        PurplePath path5 = new PurplePath("pick the up", DriveShootPickup, 5.0, 2.0);

        PurplePath path6 = new PurplePath("op the en", DriveOpen2, 5.0, 2.0);

        PurplePath path7 = new PurplePath("shoot 2: electric boogaloo", DriveOpenShoot2, 5.0, 2.0);

//        PurplePath path7 = new PurplePath("Drive Back to shoot again", DriveOpenPickup, 5.0, 2.0);

        PurplePath path8 = new PurplePath("Drive outta da trangle", RankMove, 5.0, 2.0)
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

        if (shouldShoot && !shootDone)
        {
            shootFull();
        }

        // logging
        DebugUtil.logAdd("Current Path: " + pathManager.curPath());
        DebugUtil.update();
    }

    private void flagShoot()
    {
        shouldShoot = true;
        shootTimer.reset();
        shootTimer.startTime();
    }

    private void pickupBalls(boolean should) {
        exploSwag(false, "forward");
        toggleY(should);
        explosher.setFingerState(Explosher.FingerState.STOP);
    }

    private void doBoth(boolean should) {
        shouldShoot = true;
        shootDone = false;
        shootTimer.reset();
        shootTimer.startTime();
        if (shootDone) {
            exploSwag(false, "forward");
            toggleY(should);
            explosher.setFingerState(Explosher.FingerState.STOP);
        }
    }

    private void shootFull() {
        // Start Stuff

        if (shootTimer.seconds() <= 0 ) {
            explosher.setFingerState(Explosher.FingerState.STOP);
        }
        if (shootTimer.seconds() > 2) {
            explosher.setFingerState(Explosher.FingerState.PASS);
        }
        if (shootTimer.seconds() > 0 && shootTimer.seconds() < 6.5) {
            exploSwag(true, "Forward");
        }
        if (shootTimer.seconds() > 3.5 && shootTimer.seconds() < 3.6) {
            toggleY(true);
            DebugUtil.logAdd("first push on");
        }
        if (shootTimer.seconds() > 3.6 && shootTimer.seconds() < 4.1) {
            toggleY(false);
            DebugUtil.logAdd("first push off");
        }
        if (shootTimer.seconds() > 5 && shootTimer.seconds() < 5.5) {
            toggleY(true);
            DebugUtil.logAdd("first push on");
        }
        if (shootTimer.seconds() == 5.5) {
            toggleY(false);
            DebugUtil.logAdd("first push off");
        }
        if (shootTimer.seconds() > 5.5 && shootTimer.seconds() < 6) {
            toggleX(true);
        }
        if (shootTimer.seconds() == 6) {
            toggleX(false);
        }
        if (shootTimer.seconds() > 6 && shootTimer.seconds() < 6.5) {
            toggleY(true);
        }
        if (shootTimer.seconds() >= 6.5) {
            toggleY(false);
            exploSwag(false, "Forward");
            shouldShoot = false;
            shootDone = true;
            explosher.setFingerState(Explosher.FingerState.STOP);

        }
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