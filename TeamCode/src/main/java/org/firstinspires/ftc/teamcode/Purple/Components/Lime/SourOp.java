package org.firstinspires.ftc.teamcode.Purple.Components.Lime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Limelight Distance Test", group = "PurpleTests")
public class SourOp extends OpMode
{
    private static final double METERS_TO_INCHES = 39.3701;

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init()
    {
        limelight = hardwareMap.get(Limelight3A.class, "SwagLime");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start()
    {
        limelight.start();
    }

    @Override
    public void loop()
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(yaw);

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid())
        {
            double distance = result.getBotposeAvgDist() * METERS_TO_INCHES;
            telemetry.addData("Target Dist", distance);
        }

        telemetry.update();
    }

    @Override
    public void stop()
    {
        limelight.stop();
    }
}


