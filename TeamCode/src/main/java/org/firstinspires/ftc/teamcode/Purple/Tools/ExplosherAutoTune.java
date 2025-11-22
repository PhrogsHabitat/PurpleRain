//package org.firstinspires.ftc.teamcode.Purple.Tools;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
//import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
//import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
//
//@TeleOp(name = "Explosher AutoTune", group = "Tuning")
//public class ExplosherAutoTune extends LinearOpMode
//{
//
//	private static final double[] TEST_POWERS = {0.20, 0.40, 0.60, 0.80};
//	private static final long STABILIZE_MS = 1500;
//
//	@Override
//	public void runOpMode () throws InterruptedException
//	{
//
//		DebugUtil.setTelemetry(telemetry);
//
//		Explosher explosher = new Explosher(
//				hardwareMap,
//				org.firstinspires.ftc.teamcode.Purple.Constants.FINGER_SERVO_CONFIG
//		);
//
//		waitForStart();
//
//		double[] rpmSamples = new double[TEST_POWERS.length];
//
//		telemetry.addLine("Starting kV sampling...");
//		telemetry.update();
//
//		// ------------- 1) SAMPLE kV --------------
//		for (int i = 0; i < TEST_POWERS.length; i++)
//		{
//
//			double pwr = TEST_POWERS[i];
//			explosher.setRPM(0);
//			sleep(300);
//
//			// Run motor raw power
//			explosher.setMotorState(MotorConfig.MotorState.ON);
//			explosher.setRPM(pwr * explosher.getMaxRPM());
//
//			sleep(STABILIZE_MS);
//
//			rpmSamples[i] = explosher.getCurrentRPM();
//
//			DebugUtil.logAdd("Power " + pwr + " → RPM: " + rpmSamples[i]);
//			DebugUtil.update();
//		}
//
//		// Compute least-squares kV
//		double sumNum = 0, sumDen = 0;
//
//		for (int i = 0; i < TEST_POWERS.length; i++)
//		{
//			sumNum += rpmSamples[i] * TEST_POWERS[i];
//			sumDen += rpmSamples[i] * rpmSamples[i];
//		}
//
//		double kV = (sumDen == 0) ? 0 : (sumNum / sumDen);
//
//		// Convert to your MotorConfig units
//		double finalKV = kV / 60.0;
//
//		// ------------- 2) STEP RESPONSE FOR kP --------------
//		telemetry.addLine("Measuring step response...");
//		telemetry.update();
//
//		explosher.setRPM(0);
//		sleep(500);
//
//		double targetRpm = explosher.getMaxRPM() * 0.70; // 70% step
//		explosher.setRPM(targetRpm);
//
//		double start = System.currentTimeMillis();
//		double peakError = 0;
//		double maxRPMSeen = 0;
//
//		while (opModeIsActive() && System.currentTimeMillis() - start < 1500)
//		{
//
//			double current = explosher.getCurrentRPM();
//			double err = targetRpm - current;
//
//			if (Math.abs(err) > peakError)
//				peakError = Math.abs(err);
//
//			if (current > maxRPMSeen)
//				maxRPMSeen = current;
//
//			telemetry.addData("RPM", current);
//			telemetry.update();
//		}
//
//		double overshoot = maxRPMSeen - targetRpm;
//
//		// Estimate kP (Ziegler-Nichols inspired)
//		double kP = 0.0005 + (overshoot / targetRpm) * 0.0018;
//
//		// ------------- 3) kI heuristic --------------
//		double kI = kP / 3.5;
//
//		// --------------------------------------------
//		// OUTPUT RESULTS
//		// --------------------------------------------
//
//		telemetry.addLine("=== AUTOTUNE COMPLETE ===");
//		telemetry.addData("kV", finalKV);
//		telemetry.addData("kP", kP);
//		telemetry.addData("kI", kI);
//		telemetry.addLine("Add these to MotorConfig!");
//		telemetry.update();
//
//		while (opModeIsActive())
//		{
//			sleep(50);
//		}
//	}
//}
