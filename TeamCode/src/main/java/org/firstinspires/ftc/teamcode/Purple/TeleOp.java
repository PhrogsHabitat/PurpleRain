package org.firstinspires.ftc.teamcode.Purple;

import java.util.Arrays;

import com.qualcomm.robotcore.hardware.ColorSensor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
public class TeleOp extends PurpleOpMode
{
    public static Pose startingPose;

    private static final String DRIVER_1_DRIVE_BOOST = "left_stick_button";
    private static final String DRIVER_2_EXPLORING_RESET = "a";
    private static final String DRIVER_2_FINGER_TOGGLE = "left_trigger";
    private static final String DRIVER_2_REVERSE_SHOOT = "x";
    private static final String DRIVER_2_VACUUM_FINGER_ONE = "dpad_left";
    private static final String DRIVER_2_VACUUM_FINGER_TWO = "dpad_up";
    private static final String DRIVER_2_VACUUM_OUT = "x";
    private static final String DRIVER_2_VACUUM_SHOOT = "dpad_right";
    private static final String DRIVER_2_VACUUM_IN = "y";

    private Controls driver1;
    private Controls driver2;
    private Follower follower;
    private Explosher explosher;
    private Vaccum vaccum;
    private ColorSensor colorSensor2;

    private double powerScale = Constants.DRIVE_POWER_SCALE;
    private boolean wasTagDetected = false;
    private Explosher.FingerState fingerState = Explosher.FingerState.STOP;

    @Override
    public void create()
    {
        driver1 = new Controls(gamepad1);
        driver2 = new Controls(gamepad2);
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        follower.startTeleopDrive(true);

        PurpleMemory.initialize(hardwareMap);
        LimeUtil.start(hardwareMap, 60);
        LimeUtil.setPipeline(0);

        explosher = new Explosher(hardwareMap);
        vaccum = new Vaccum(hardwareMap);
        colorSensor2 = hardwareMap.get(ColorSensor.class, Names.COLOR2);

        DebugUtil.setTelemetry(telemetry);
    }

    @Override
    public void update()
    {
        driver1.update();
        driver2.update();
        follower.update();
        PurpleMemory.Instance.update();

        LimeUtil.update();
        explosher.update();
        vaccum.update();

        updateAprilTagFeedback();
        updateDrive();
        updateExplosher();
        updateVaccum();
        teleInfo();
    }

    @Override
    public void destroy()
    {
        explosher.stop();
        vaccum.stop();
    }

    private void updateAprilTagFeedback()
    {
        boolean tagDetected = LimeUtil.hasValidTarget();
        if (tagDetected && !wasTagDetected)
        {
            driver1.vibrate(150);
            driver2.vibrate(Constants.VIBRATION_TAG_DETECTED);
        }

        wasTagDetected = tagDetected;
    }

    private void updateDrive()
    {
        powerScale = driver1.isPressed(DRIVER_1_DRIVE_BOOST) ? Constants.DRIVE_POWER_BOOST : Constants.DRIVE_POWER_SCALE;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * powerScale,
                -gamepad1.left_stick_x * powerScale,
                -gamepad1.right_stick_x * powerScale,
                true
        );
    }

    private void updateExplosher()
    {
        double leftStickY = driver2.getLeftStickY();
        if (leftStickY > Constants.JOYSTICK_DEADZONE)
        {
            if (LimeUtil.getTargetDistance() != 0)
            {
                explosher.setRegressionEnabled(true);
                explosher.setRPM(-explosher.getSmoothedTargetRPM());
            }
            else
            {
                explosher.setRegressionEnabled(false);
                explosher.stop();
            }
        }
        else if (leftStickY < -Constants.JOYSTICK_DEADZONE && driver2.isPressed(DRIVER_2_REVERSE_SHOOT))
        {
            explosher.setRegressionEnabled(false);
            explosher.setRPM(4000);
        }
        else
        {
            explosher.setRegressionEnabled(false);
            explosher.stop();
        }

        explosher.updateAim(follower.getPose());
        updateFingerState();

        if (driver2.isPressed(DRIVER_2_EXPLORING_RESET))
        {
            explosher.resetExploringPos();
        }
    }

    private void updateFingerState()
    {
        if (!driver2.justPressed(DRIVER_2_FINGER_TOGGLE))
        {
            return;
        }

        fingerState = fingerState == Explosher.FingerState.PASS ? Explosher.FingerState.STOP : Explosher.FingerState.PASS;
        explosher.setFingerState(fingerState);
    }

    private void updateVaccum()
    {
        if (driver2.justPressed(DRIVER_2_VACUUM_FINGER_ONE))
        {
            vaccum.flickFinger(0);
        }

        if (driver2.justPressed(DRIVER_2_VACUUM_FINGER_TWO))
        {
            vaccum.flickFinger(1);
        }

        if (driver2.justPressed(DRIVER_2_VACUUM_SHOOT))
        {
            vaccum.shoot();
        }

        if (driver2.isPressed(DRIVER_2_VACUUM_IN))
        {
            vaccum.setPower(Vaccum.DEFAULT_POW);
        }
        else if (driver2.isPressed(DRIVER_2_VACUUM_OUT))
        {
            vaccum.setPower(-Vaccum.DEFAULT_POW);
        }
        else
        {
            vaccum.stop();
        }
    }

    private void teleInfo()
    {
        DebugUtil.logAdd("============== [LIME]");
        DebugUtil.logAdd(" ");
        DebugUtil.logAdd("Target X: " + LimeUtil.getTx());
        DebugUtil.logAdd("Target D: " + LimeUtil.getTargetDistance());
        DebugUtil.logAdd(" ");

        DebugUtil.logAdd("============== [EXPLOSHER]");
        DebugUtil.logAdd(" ");
        DebugUtil.logAdd("Target RPM: " + explosher.getTargetRPM());
        DebugUtil.logAdd("Current RPM: " + explosher.getCurrentRPM());
        DebugUtil.logAdd("Smoothed Regress: " + explosher.getSmoothedTargetRPM());
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

        DebugUtil.logAdd("============== [COLOR]");
        DebugUtil.logAdd(" ");
        DebugUtil.logAdd(String.format(
                "Sensor2 RGB: R=%d G=%d B=%d",
                colorSensor2.red(),
                colorSensor2.green(),
                colorSensor2.blue()
        ));
        DebugUtil.logAdd(" ");

        DebugUtil.logAdd("============== [MEMORY]");
        DebugUtil.logAdd("[MOTIF]: " + PurpleMemory.Instance.curMotif());
        DebugUtil.logAdd("[BALLS]: " + Arrays.toString(PurpleMemory.Instance.curBalls()));
        DebugUtil.logAdd(" ");

        Pose followerPose = follower.getPose();
        DebugUtil.logAdd("[POSITION] HEADING: " + Math.toDegrees(followerPose.getHeading()));
        DebugUtil.logAdd("[POSITION] X: " + followerPose.getX());
        DebugUtil.logAdd("[POSITION] Y: " + followerPose.getY());
        DebugUtil.logAdd(" ");

        DebugUtil.update();
    }
}

