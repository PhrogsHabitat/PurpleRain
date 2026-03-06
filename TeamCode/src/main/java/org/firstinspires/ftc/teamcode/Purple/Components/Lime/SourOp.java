package org.firstinspires.ftc.teamcode.Purple.Components.Lime;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "Limelight Distance Test", group = "Tests")
public class SourOp extends OpMode
{

	// Declare the Limelight3A object
	private Limelight3A limelight;

	// Declare the IMU for robot orientation
	private IMU imu;

	@Override
	public void init ()
	{

		// Lime
		limelight = hardwareMap.get(Limelight3A.class, "SwagLime");
		limelight.pipelineSwitch(0);

		// IMU
		imu = hardwareMap.get(IMU.class, "imu");

		// Paramters apparently exist
		RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
		RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

		RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

		imu.initialize(new IMU.Parameters(orientationOnRobot));

		telemetry.addData("Status", "Initialized");
		telemetry.update();
	}

	@Override
	public void start ()
	{
		limelight.start();
	}

	@Override
	public void loop ()
	{

		// Get orientation from control hub
		YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
		double yaw = orientation.getYaw(AngleUnit.DEGREES);

		// Tell Lime our orientation
		limelight.updateRobotOrientation(yaw);

		// Get the latest result from the Limelight
		LLResult llResult = limelight.getLatestResult();

		// Check if the result is valid and a target is detected
		if (llResult != null && llResult.isValid())
		{

			// Bruh we had this method the whole time?
			double dist = llResult.getBotposeAvgDist() * 39.3701;
			Pose3D MT1 = llResult.getBotpose();
			Pose3D MT2 = llResult.getBotpose_MT2();

			telemetry.addData("Target Dist", dist);

		}
		else
		{

		}

		telemetry.update();
	}

	@Override
	public void stop ()
	{

	}
}
