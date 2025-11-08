package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.LimeUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PurpleTeleOp", group = "Purple")
public class TeleOp extends LinearOpMode {
    private Controls driver1;
    private Controls driver2;

    private DcMotorEx fl, fr, bl, br;
    private double powerScale = Constants.DRIVE_POWER_SCALE;
    private Explosher explosher;
    private Vaccum vaccum;

    // State tracking
    private boolean wasAligned = false;
    private boolean wasTagDetected = false;
    private Explosher.DistanceState stickyState = null;

    // Auto-align state
    private boolean autoAlignActive = false;
    private long lastTagSeenTime = 0;
    private static final long TAG_TIMEOUT_MS = 500;

    @Override
    public void runOpMode() {
        driver1 = new Controls(gamepad1);
        driver2 = new Controls(gamepad2);

        initializeMotors();
        initializeExplosher();
        initializeVaccum();
        DebugUtil.setTelemetry(telemetry);

        waitForStart();
        while (opModeIsActive()) {
            driver1.update();
            driver2.update();
            update();
        }
        stopAll();
    }

    private void initializeMotors() {
        fl = hardwareMap.get(DcMotorEx.class, Constants.FRONT_LEFT_MOTOR);
        fr = hardwareMap.get(DcMotorEx.class, Constants.FRONT_RIGHT_MOTOR);
        bl = hardwareMap.get(DcMotorEx.class, Constants.BACK_LEFT_MOTOR);
        br = hardwareMap.get(DcMotorEx.class, Constants.BACK_RIGHT_MOTOR);

        MotorConfig[] configs = {Constants.FL_CONFIG, Constants.FR_CONFIG, Constants.BL_CONFIG, Constants.BR_CONFIG};
        DcMotorEx[] motors = {fl, fr, bl, br};
        for (int i = 0; i < motors.length; i++) {
            motors[i].setDirection(configs[i].getDirection());
            motors[i].setZeroPowerBehavior(configs[i].getZeroPowerBehavior());
            motors[i].setMode(configs[i].getRunMode());
        }
    }

    private void initializeExplosher() {
        LimeUtil.start(hardwareMap, "SwagLime", 60);
        LimeUtil.setPipeline(0);
        explosher = new Explosher(
                hardwareMap.get(DcMotorEx.class, Constants.EXPLOSHER_MOTOR),
                Constants.EXPLOSHER_CONFIG,
                hardwareMap.get(Servo.class, Constants.FINGER_SERVO),
                Constants.FINGER_SERVO_CONFIG
        );
    }

    private void initializeVaccum() {
        vaccum = new Vaccum(
                hardwareMap.get(DcMotorEx.class, Constants.INTAKE_MOTOR),
                Constants.INTAKE_CONFIG,
                hardwareMap.get(DcMotorEx.class, Constants.MIDTAKE_MOTOR),
                Constants.MIDTAKE_CONFIG
        );
    }

    private void update() {
        updateDrive();
        updateAprilTagFeedback();
        updatePlayer1Controls();
        updatePlayer2Controls();
        updateSubsystems();
        updateTelemetry();
    }

    private void updateDrive() {
        // Speed boost when left stick is pressed
        powerScale = driver1.isPressed("left_stick_button") ?
                Constants.DRIVE_POWER_BOOST : Constants.DRIVE_POWER_SCALE;

        double forward = -driver1.getLeftStickY();
        double strafe = driver1.getLeftStickX();
        double turn = driver1.getRightStickX();

        double[] powers = MotorUtil.normalizePowers(new double[]{
                (-forward - strafe - turn),
                (-forward + strafe - turn),
                (forward - strafe - turn),
                (forward + strafe - turn)
        });

        fl.setPower(powers[0] * powerScale);
        bl.setPower(powers[1] * powerScale);
        fr.setPower(powers[2] * powerScale);
        br.setPower(powers[3] * powerScale);
    }

    private void updateAprilTagFeedback() {
        boolean tagDetected = LimeUtil.hasValidTarget();

        // Quick vibration when tag is detected
        if (tagDetected && !wasTagDetected) {
            driver1.vibrate(Constants.VIBRATION_TAG_DETECTED);
            driver2.vibrate(Constants.VIBRATION_TAG_DETECTED);
        }

        wasTagDetected = tagDetected;
    }

    private void updatePlayer1Controls() {
        // Auto-align with AprilTag when right bumper held
        if (driver1.isPressed("right_bumper") && LimeUtil.hasValidTarget()) {
            autoAlignToTag();
        }
        else if (!driver1.isPressed("right_bumper"))
        {
            autoAlignActive = false;
        }

        // Impact detection
        checkForImpact();
    }

    private void updatePlayer2Controls() {
        updateExplosherStickControl();
        updateExplosherTriggerControl();
        updateVaccumControl();

        // Vibration when fully aligned with AprilTag
        if (isFullyAligned() && !wasAligned) {
            driver2.vibrate(Constants.VIBRATION_ALIGNED);
        }
        wasAligned = isFullyAligned();
    }

    private void updateExplosherStickControl() {
        double leftStickY = driver2.getLeftStickY();

        if (Math.abs(leftStickY) > Constants.JOYSTICK_DEADZONE) {
            // Stick is being used - clear sticky state
            stickyState = null;

            if (driver2.isPressed("left_stick_button")) {
                // Stick pressed + forward = FAR sweet spot
                explosher.setRPM(Constants.EXPLOSHER_FAR_SWEET);
                explosher.setMotorState(MotorConfig.MotorState.ON);
            } else if (leftStickY > 0.5) {
                // Stick forward = CLOSE sweet spot
                explosher.setRPM(Constants.EXPLOSHER_CLOSE_SWEET);
                explosher.setMotorState(MotorConfig.MotorState.ON);
            }
        } else if (stickyState == null) {
            // Stick returned to center and no sticky state - turn off
            explosher.setMotorState(MotorConfig.MotorState.OFF);
        }
    }

    private void updateExplosherTriggerControl() {
        // Left trigger cycles sweet spots in sticky mode
        if (driver2.justPressed("left_trigger")) {
            if (stickyState == null || stickyState == Explosher.DistanceState.FAR) {
                stickyState = Explosher.DistanceState.NEAR;
                explosher.setRPM(Constants.EXPLOSHER_CLOSE_SWEET);
            } else {
                stickyState = Explosher.DistanceState.FAR;
                explosher.setRPM(Constants.EXPLOSHER_FAR_SWEET);
            }
            explosher.setMotorState(MotorConfig.MotorState.ON);
        }
    }

    private void updateVaccumControl() {
        if (driver2.isPressed("y")) {
            vaccum.setState(MotorConfig.MotorState.ON);
        } else if (driver2.isPressed("x")) {
            vaccum.setState(MotorConfig.MotorState.ON);
            vaccum.setRPM(-Vaccum.DEFAULT_RPM);
        } else {
            vaccum.setState(MotorConfig.MotorState.OFF);
        }
    }

    private boolean hasRecentTarget() {
        return LimeUtil.hasValidTarget() &&
                (System.currentTimeMillis() - lastTagSeenTime) < TAG_TIMEOUT_MS;
    }

    private void autoAlignToTag() {
        if (!hasRecentTarget()) {
            // No recent target, disable auto-align and return control to driver
            autoAlignActive = false;
            return;
        }

        double tx = LimeUtil.getTx(); // Horizontal offset from center (-degrees to +degrees)
        double distance = LimeUtil.getTargetDistance(); // Distance to target

        // Calculate errors
        double angleError = -tx; // Negative because we want to move opposite to the error
        double distanceError = Constants.DESIRED_TAG_DISTANCE - distance;

        // Apply deadzone to prevent jitter
        if (Math.abs(angleError) < Constants.ALIGN_ANGLE_DEADZONE) {
            angleError = 0;
        }
        if (Math.abs(distanceError) < Constants.ALIGN_DISTANCE_DEADZONE) {
            distanceError = 0;
        }

        // Calculate correction powers with clamping
        double strafePower = clamp(angleError * Constants.ALIGN_ANGLE_KP,
                -Constants.MAX_ALIGN_POWER, Constants.MAX_ALIGN_POWER);

        double forwardPower = 0;

        // Allow manual turning during auto-align
        double turnPower = clamp(-angleError * Constants.ALIGN_ANGLE_KP,
                -Constants.MAX_ALIGN_POWER, Constants.MAX_ALIGN_POWER);

        // Apply the corrections - Mecanum wheel calculations
        double[] powers = MotorUtil.normalizePowers(new double[]{
                (-forwardPower - strafePower - turnPower),
                (-forwardPower + strafePower - turnPower),
                (forwardPower - strafePower - turnPower),
                (forwardPower + strafePower - turnPower)
        });

        fl.setPower(powers[0] * powerScale);
        bl.setPower(powers[1] * powerScale);
        fr.setPower(powers[2] * powerScale);
        br.setPower(powers[3] * powerScale);

        // Progressive vibration feedback based on alignment quality
        double alignmentError = Math.abs(angleError) + Math.abs(distanceError);
        if (alignmentError < 2.0) {
            // Fully aligned - continuous gentle vibration
            driver1.vibrate(50);
        } else if (alignmentError < 5.0) {
            // Close - pulsed vibration
            if ((System.currentTimeMillis() % 500) < 250) {
                driver1.vibrate(25);
            }
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private boolean isFullyAligned() {
        if (!hasRecentTarget()) return false;

        double tx = LimeUtil.getTx();
        double distance = LimeUtil.getTargetDistance();
        double distanceError = Math.abs(distance - Constants.DESIRED_TAG_DISTANCE);

        return Math.abs(tx) < Constants.ALIGN_ANGLE_TOLERANCE &&
                distanceError < Constants.ALIGN_DISTANCE_TOLERANCE;
    }

    private void checkForImpact() {
        // Simple impact detection
        double totalPower = Math.abs(fl.getPower()) + Math.abs(fr.getPower()) +
                Math.abs(bl.getPower()) + Math.abs(br.getPower());

        // This is a placeholder - implement proper impact detection with encoders/IMU
        if (totalPower > 0.5) {
            // Potential impact detection logic here
        }
    }

    private void updateSubsystems() {
        explosher.update();
        vaccum.update();
    }

    private void updateTelemetry() {
        DebugUtil.logAdd("Power Scale: " + powerScale);
        DebugUtil.logAdd("Auto-Align: " + (autoAlignActive ? "ACTIVE" : "INACTIVE"));
        DebugUtil.logAdd("Sticky State: " + stickyState);
        if (LimeUtil.hasValidTarget()) {
            DebugUtil.logAdd("AprilTag - Dist: " + String.format("%.1f", LimeUtil.getTargetDistance()) +
                    "in, Angle: " + String.format("%.1f", LimeUtil.getTx()) + "°");
            DebugUtil.logAdd("Aligned: " + (isFullyAligned() ? "YES" : "NO"));
        } else {
            DebugUtil.logAdd("AprilTag: No target");
        }
        DebugUtil.logAdd("Explosher RPM: " + String.format("%.1f", explosher.getCurrentRPM()));
        DebugUtil.update();
    }

    private void stopAll() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        explosher.setMotorState(MotorConfig.MotorState.OFF);
        vaccum.setState(MotorConfig.MotorState.OFF);
        autoAlignActive = false;
    }
}