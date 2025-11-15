//package org.firstinspires.ftc.teamcode.Purple;
//
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
//import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
//import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;
//import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
//import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
//import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
//import org.firstinspires.ftc.teamcode.Purple.Utils.LimeUtil;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "PurpleAuto", group = "Purple")
//public class AutoOp extends LinearOpMode
//{
//    private Controls driver1;
//    private Controls driver2;
//
//    private DcMotorEx fl, fr, bl, br;
//    private double powerScale = Constants.DRIVE_POWER_SCALE;
//    private Explosher explosher;
//    private Vaccum vaccum;
//    private boolean prevA = false;
//    private boolean explosherToggled = false;
//    private boolean prevX = false;
//    private boolean prevRightBumper = false;
//    private boolean prevLeftBumper = false;
//    private ElapsedTime timer = new ElapsedTime();
//
//
//    @Override
//    public void runOpMode()
//    {
//        driver1 = new Controls(gamepad1);
//        driver2 = new Controls(gamepad2);
//
//        initializeMotors();
//        initializeExplosher();
//        initializeVaccum();
//        DebugUtil.setTelemetry(telemetry);
//        waitForStart();
//
//        timer.reset();
//        while (opModeIsActive())
//        {
//            update();
//        }
//        stopMotors();
//        explosher.setMotorState(MotorConfig.MotorState.OFF);
//    }
//
//    private void initializeMotors()
//    {
//        fl = hardwareMap.get(DcMotorEx.class, Constants.FRONT_LEFT_MOTOR);
//        fr = hardwareMap.get(DcMotorEx.class, Constants.FRONT_RIGHT_MOTOR);
//        bl = hardwareMap.get(DcMotorEx.class, Constants.BACK_LEFT_MOTOR);
//        br = hardwareMap.get(DcMotorEx.class, Constants.BACK_RIGHT_MOTOR);
//
//        MotorConfig[] configs = {Constants.FL_CONFIG, Constants.FR_CONFIG, Constants.BL_CONFIG, Constants.BR_CONFIG};
//        DcMotorEx[] motors = {fl, fr, bl, br};
//        for (int i = 0; i < motors.length; i++)
//        {
//            motors[i].setDirection(configs[i].getDirection());
//            motors[i].setZeroPowerBehavior(configs[i].getZeroPowerBehavior());
//            motors[i].setMode(configs[i].getRunMode());
//        }
//    }
//
//    private void initializeExplosher()
//    {
//        LimeUtil.start(hardwareMap, "SwagLime", 60);
//        LimeUtil.setPipeline(0);
//        explosher = new Explosher(
//                hardwareMap.get(DcMotorEx.class, Constants.EXPLOSHER_MOTOR),
//                Constants.EXPLOSHER_CONFIG,
//                hardwareMap.get(Servo.class, Constants.FINGER_SERVO),
//                Constants.FINGER_SERVO_CONFIG
//        );
//
//    }
//
//    private void initializeVaccum() {
//        vaccum = new Vaccum(
//                hardwareMap.get(DcMotorEx.class, Constants.INTAKE_MOTOR),
//                Constants.INTAKE_CONFIG,
//                hardwareMap.get(DcMotorEx.class, Constants.MIDTAKE_MOTOR),
//                Constants.MIDTAKE_CONFIG
//        );
//    }
//
//    private void update()
//    {
//        double elapsed = timer.seconds();
//
//        // Fixed drive code - using standard mecanum equations
//        double forward = 0;
//        double strafe = 0;
//        double turn = 0;
//
//        if (elapsed < 2.25)
//        {
//            forward = -0.4;
//        }
//
//        if (elapsed > 3)
//        {
//            explosher.setMotorState(MotorConfig.MotorState.ON);
//            explosher.setRPM(explosher.CLOSE_SWEET);
//        }
//
//        if (elapsed > 6 && elapsed < 16)
//        {
//            vaccum.setState(MotorConfig.MotorState.ON);
//        }
//
//        if (elapsed > 16 && elapsed < 18)
//        {
//            strafe = 0.4;
//            vaccum.setState(MotorConfig.MotorState.OFF);
//            explosher.setMotorState(MotorConfig.MotorState.OFF);
//        }
//
//        if (elapsed > 18)
//        {
//            forward = 0;
//            strafe = 0;
//            turn = 0;
//
//            vaccum.setState(MotorConfig.MotorState.OFF);
//            explosher.setMotorState(MotorConfig.MotorState.OFF);
//        }
//
//        double[] powers = MotorUtil.normalizePowers(new double[]{
//                (-forward - strafe - turn),
//                (-forward + strafe - turn),
//                (forward - strafe - turn),
//                (forward + strafe - turn)
//        });
//        fl.setPower(powers[0] * powerScale);
//        bl.setPower(powers[1] * powerScale);
//        fr.setPower(powers[2] * powerScale);
//        br.setPower(powers[3] * powerScale);
//
//        // LimeLight distance detection and printing
//        if (LimeUtil.hasValidTarget()) {
//            double distance = LimeUtil.getTargetDistance();
//            DebugUtil.logAdd("AprilTag Distance: " + String.format("%.2f", distance) + " inches");
//        } else {
//            DebugUtil.logAdd("No AprilTag detected");
//        }
//
//        // RPM debug controls
//        boolean currentRightBumper = gamepad2.right_bumper;
//        boolean currentLeftBumper = gamepad2.left_bumper;
//
//        if (currentRightBumper && !prevRightBumper) {
//            // Increase RPM by 5
//            double newRPM = explosher.getTargetRPM() + 5;
//            explosher.setRPM(newRPM);
//            DebugUtil.logAdd("RPM Increased to: " + newRPM);
//        }
//
//        if (currentLeftBumper && !prevLeftBumper && explosher.getTargetRPM() >= 5) {
//            // Decrease RPM by 5 (but not below 0)
//            double newRPM = explosher.getTargetRPM() - 5;
//            explosher.setRPM(newRPM);
//            DebugUtil.logAdd("RPM Decreased to: " + newRPM);
//        }
//
//        prevRightBumper = currentRightBumper;
//        prevLeftBumper = currentLeftBumper;
//
//        // Existing controls
//        if (driver2.justPressed("dpad_up"))
//        {
//            explosher.setMotorState(MotorConfig.MotorState.ON);
//            explosher.setRPM(explosher.CLOSE_SWEET);
//            DebugUtil.logAdd("RPM set to CLOSE_SWEET: " + explosher.CLOSE_SWEET);
//        }
//        if (driver2.justPressed("dpad_down"))
//        {
//            explosher.setMotorState(MotorConfig.MotorState.ON);
//            explosher.setRPM(explosher.FAR_SWEET);
//            DebugUtil.logAdd("RPM set to FAR_SWEET: " + explosher.FAR_SWEET);
//        }
//        if (driver2.justPressed("dpad_left"))
//        {
//            explosher.setMotorState(MotorConfig.MotorState.OFF);
//        }
//
//        // EXPLOSHER control
//        if (Constants.DEBUG_MODE)
//        {
////            if (gamepad2.a && !prevA)
////            {
////                explosherToggled = !explosherToggled;
////                explosher.setMotorState(explosherToggled ? MotorConfig.MotorState.ON : MotorConfig.MotorState.OFF);
////            }
////            prevA = gamepad2.a;
//        } else
//        {
//            if (gamepad2.b)
//            {
//                explosher.setMotorState(MotorConfig.MotorState.ON);
//            } else
//            {
//                explosher.setMotorState(MotorConfig.MotorState.OFF);
//            }
//        }
//
//        // Cycle Explosher distance state with X button (edge detection)
//        if (gamepad2.x && !prevX)
//        {
//            explosher.cycleDistanceState();
//        }
//
//        prevX = gamepad2.x;
//
//        // Vaccum control (using Y button as a hold instead of left bumper)
//        if (gamepad2.y) {
//            vaccum.setState(MotorConfig.MotorState.ON);
//        } else {
//            vaccum.setState(MotorConfig.MotorState.OFF);
//        }
//
//        explosher.update();
//        vaccum.update();
//
//        DebugUtil.logAdd("FL: " + powers[0] + ", FR: " + powers[1] + ", BL: " + powers[2] + ", BR: " + powers[3]);
//        DebugUtil.logAdd("Finger: " + explosher.getFingerPosition());
//        DebugUtil.logAdd("Current Explosher RPM: " + String.format("%.2f", explosher.getCurrentRPM()));
//        DebugUtil.logAdd("Target Explosher RPM: " + String.format("%.2f", explosher.getTargetRPM()));
//        DebugUtil.update();
//    }
//
//    private void stopMotors()
//    {
//        fl.setPower(0);
//        fr.setPower(0);
//        bl.setPower(0);
//        br.setPower(0);
//        explosher.setMotorState(MotorConfig.MotorState.OFF);
//        explosher.setFingerState(ServoConfig.ServoState.OFF);
//        vaccum.setState(MotorConfig.MotorState.OFF);
//    }
//}