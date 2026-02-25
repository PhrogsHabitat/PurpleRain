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
			.mass(23.0)
			.forwardZeroPowerAcceleration(-32.52225256705156)
			.lateralZeroPowerAcceleration(-50.2725693616272);

	public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

	public static MecanumConstants driveConstants = new MecanumConstants()
			.maxPower(1)
			.xVelocity(68.47436859851749)
			.yVelocity(54.857007124292565)

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
			.forwardPodY(7.5)
			.strafePodX(-2.3)
			.distanceUnit(DistanceUnit.INCH)
			.hardwareMapName("odo")
			.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
			.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
			.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

	public static Follower createFollower (HardwareMap hardwareMap)
	{

		return new FollowerBuilder(followerConstants, hardwareMap)
				.pathConstraints(pathConstraints)
				.mecanumDrivetrain(driveConstants)
				.pinpointLocalizer(localizerConstants)
				.build();
	}

}
