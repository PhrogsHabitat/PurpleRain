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

        // Impact detection (simplified - monitors sudden motor power changes)
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
        double leftStickY = -driver2.getLeftStickY(); // Invert for natural feel

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

            // Simulate adaptive trigger resistance (would need custom hardware support)
            driver2.setTriggerFeedback(stickyState == Explosher.DistanceState.FAR ? 0.5f : 1.0f);
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

    private void autoAlignToTag() {
        if (!LimeUtil.hasValidTarget()) return;

        double tx = LimeUtil.getTx();
        double distance = LimeUtil.getTargetDistance();

        // Simple P-controller for alignment
        double strafeCorrection = tx * Constants.ALIGN_KP;
        double distanceError = distance - Constants.ALIGN_POSITION_TOLERANCE;
        double forwardCorrection = Math.max(-0.3, Math.min(0.3, distanceError * 0.01));

        // Apply corrections
        double[] powers = MotorUtil.normalizePowers(new double[]{
                (-forwardCorrection - strafeCorrection),
                (-forwardCorrection + strafeCorrection),
                (forwardCorrection - strafeCorrection),
                (forwardCorrection + strafeCorrection)
        });

        fl.setPower(powers[0] * powerScale);
        bl.setPower(powers[1] * powerScale);
        fr.setPower(powers[2] * powerScale);
        br.setPower(powers[3] * powerScale);

        // Vibration feedback based on alignment error
        double alignmentError = Math.abs(tx) + Math.abs(distanceError);
        if (alignmentError > 5) {
            driver1.vibrate((int)(alignmentError * 10)); // More intense when far from target
        }
    }

    private boolean isFullyAligned() {
        if (!LimeUtil.hasValidTarget()) return false;

        double tx = LimeUtil.getTx();
        double distance = LimeUtil.getTargetDistance();
        double distanceError = Math.abs(distance - Constants.ALIGN_POSITION_TOLERANCE);

        return Math.abs(tx) < Constants.ALIGN_ANGLE_TOLERANCE &&
                distanceError < Constants.ALIGN_POSITION_TOLERANCE;
    }

    private void checkForImpact() {
        // Simple impact detection - monitor motor power vs actual movement
        // This is a simplified version - you might want to use current sensing or encoders
        double totalPower = Math.abs(fl.getPower()) + Math.abs(fr.getPower()) +
                Math.abs(bl.getPower()) + Math.abs(br.getPower());

        // If motors are trying to move but robot isn't (detected via encoders or IMU)
        // This would need proper implementation with your odometry system
        if (totalPower > 0.5) { // Arbitrary threshold
            driver1.vibrate(Constants.VIBRATION_IMPACT);
        }
    }

    private void updateSubsystems() {
        explosher.update();
        vaccum.update();
    }

    private void updateTelemetry() {
        DebugUtil.logAdd("Power Scale: " + powerScale);
        DebugUtil.logAdd("Sticky State: " + stickyState);
        if (LimeUtil.hasValidTarget()) {
            DebugUtil.logAdd("AprilTag Distance: " + String.format("%.2f", LimeUtil.getTargetDistance()) + " inches");
        }
        DebugUtil.logAdd("Explosher RPM: " + String.format("%.2f", explosher.getCurrentRPM()));
        DebugUtil.update();
    }

    private void stopAll() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        explosher.setMotorState(MotorConfig.MotorState.OFF);
        vaccum.setState(MotorConfig.MotorState.OFF);
    }
}