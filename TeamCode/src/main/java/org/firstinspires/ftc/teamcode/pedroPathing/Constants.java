package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants
{

	public static FollowerConstants followerConstants = new FollowerConstants()
			.mass(17.8)
			.forwardZeroPowerAcceleration(-27.775195443276083)
			.lateralZeroPowerAcceleration(-56.30049389816956);

	public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .1, 5);

	public static MecanumConstants driveConstants = new MecanumConstants()
			.maxPower(1)
			.xVelocity(64.69473290631153)
			.yVelocity(50.010922890009844)

			.rightFrontMotorName("FR")
			.rightRearMotorName("BR")
			.leftRearMotorName("BL")
			.leftFrontMotorName("FL")
			.leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
			.leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
			.rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
			.rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
			.useBrakeModeInTeleOp(true);

	public static PinpointConstants localizerConstants = new PinpointConstants()
			.forwardPodY(-5.3)
			.strafePodX(-2.5)
			.distanceUnit(DistanceUnit.INCH)
			.hardwareMapName("odo")
			.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
			.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
			.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

	public static Follower createFollower (HardwareMap hardwareMap)
	{

		return new FollowerBuilder(followerConstants, hardwareMap)
				.pathConstraints(pathConstraints)
				.mecanumDrivetrain(driveConstants)
				.pinpointLocalizer(localizerConstants)
				.build();
	}

}
