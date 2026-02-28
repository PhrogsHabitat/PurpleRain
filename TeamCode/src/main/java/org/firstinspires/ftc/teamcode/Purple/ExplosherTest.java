package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;

/**
 * Simple teleop for testing the Explosher subsystem only.
 *
 * Controls (gamepad1):
 * - Left stick Y : proportional RPM (hold to set). Small deadzone.
 * - A            : set preset RPM 3000
 * - X            : set preset RPM 4000
 * - B            : stop shooter (set RPM 0)
 * - Y            : set preset RPM 1000
 * - Right bumper : cycle finger state (rising edge)
 * - Dpad up/down : adjust debug finger position (small increments)
 * - Dpad left    : reset exploring encoder to zero
 * - Left bumper  : exploring +5 degrees (rising edge)
 * - Right bumper (long press) also cycles finger, but is edge-detected to avoid spamming
 */
@TeleOp(name = "ExplosherTest", group = "Purple")
public class ExplosherTest extends OpMode
{
    private static final double JOYSTICK_DEADZONE = 0.08;
    private static final double DEBUG_FINGER_INCREMENT = 0.01;

    private Explosher explosher;

    // edge detection helpers
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void init()
    {
        explosher = new Explosher(hardwareMap);
        // start stopped
        explosher.stop();

        telemetry.addData("Status", "Explosher test initialized");
    }

    @Override
    public void loop()
    {
        // Read joystick for proportional RPM control
        double stickY = -gamepad1.left_stick_y; // forward = positive

        if (Math.abs(stickY) > JOYSTICK_DEADZONE)
        {
            double rpm = stickY * explosher.getMaxRPM();
            explosher.setRPM(rpm);
            telemetry.addData("Control", "Joystick RPM control");
        }
        else
        {
            // Preset buttons
            if (gamepad1.a)
            {
                explosher.setRPM(4000);
                telemetry.addData("Control", "Preset 3000 RPM");
            }
            else if (gamepad1.x)
            {
                explosher.setRPM(-4000);
                telemetry.addData("Control", "Preset 4000 RPM");
            }
            else if (gamepad1.y)
            {
                explosher.setRPM(1000);
                telemetry.addData("Control", "Preset 1000 RPM");
            }
            else if (gamepad1.b)
            {
                explosher.stop();
                telemetry.addData("Control", "Stopped");
            }
        }

        // Edge-detect right bumper to cycle finger state
        if (gamepad1.right_bumper && !prevRightBumper)
        {
            explosher.cycleFingerState();
        }

        // Dpad up/down to adjust debug finger position (edge detect)
        if (gamepad1.dpad_up && !prevDpadUp)
        {
            explosher.adjustDebugFingerPosition(DEBUG_FINGER_INCREMENT);
        }

        if (gamepad1.dpad_down && !prevDpadDown)
        {
            explosher.adjustDebugFingerPosition(-DEBUG_FINGER_INCREMENT);
        }

        // Dpad left resets exploring encoder
        if (gamepad1.dpad_left)
        {
            explosher.resetExploringPos();
        }

        // Left bumper increases exploring angle by +5 degrees (edge detect)
        if (gamepad1.left_bumper && !prevLeftBumper)
        {
            double curDeg = explosher.getExploringDeg();
            explosher.setExploringDeg(curDeg + 5.0);
        }

        // Update previous button states for edge detection
        prevRightBumper = gamepad1.right_bumper;
        prevLeftBumper = gamepad1.left_bumper;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;

        // Always call update to let Explosher internals run
        explosher.update();

        // Telemetry
        telemetry.addData("Current RPM", String.format("%.1f", explosher.getCurrentRPM()));
        telemetry.addData("Target RPM", String.format("%.1f", explosher.getTargetRPM()));
        telemetry.addData("Max RPM", String.format("%.1f", explosher.getMaxRPM()));
        telemetry.addData("Finger State", explosher.getFingerStateEnum().toString());
        telemetry.addData("Finger Pos", String.format("%.3f", explosher.getFingerPosition()));
        telemetry.addData("Exploring Deg", String.format("%.2f", explosher.getExploringDeg()));

        telemetry.update();
    }

    @Override
    public void stop()
    {
        if (explosher != null)
        {
            explosher.stop();
        }
    }
}
