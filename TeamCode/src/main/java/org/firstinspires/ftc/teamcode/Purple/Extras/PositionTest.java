package org.firstinspires.ftc.teamcode.Purple.Extras;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Controls;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(
		name = "PositionTest",
		group = "Purple"
)
public class PositionTest extends PurpleOpMode
{
	// ---- CONFIG ----
	private static final Pose DEFAULT_START_POSE =
			new Pose(72, 72, Math.toRadians(0.0));

	// Max allowed difference (inches) between odometry and vision
	private static final double MAX_VISION_CORRECTION_DISTANCE = 5.0;

	// -----------------

	private Follower follower;
	private Controls driver1;

	@Override
	public void create ()
	{

		driver1 = new Controls(gamepad1);

		// ---- PEDRO FOLLOWER ----
		follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants
				.createFollower(hardwareMap);

		follower.setStartingPose(DEFAULT_START_POSE);
		follower.update();
		follower.startTeleopDrive(true);

		// ---- LIMELIGHT ----
		LimeUtil.start(hardwareMap, 60);
		LimeUtil.setPipeline(0);

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update ()
	{

		driver1.update();

		// Always update follower first
		follower.update();

		// Drive robot
		updateDrive();

		// Update LimeUtil (VERY IMPORTANT)
		LimeUtil.update();

		// Output telemetry
		teleInfo();
	}

	@Override
	public void destroy ()
	{
		// nothing special
	}

	private void updateDrive ()
	{

		follower.setTeleOpDrive(
				-gamepad1.left_stick_y,
				-gamepad1.left_stick_x,
				-gamepad1.right_stick_x,
				true
		);
	}

	private void teleInfo ()
	{

		Pose pose = follower.getPose();

		DebugUtil.logAdd("========== POSITION TEST ==========");
		DebugUtil.logAdd(" ");

		Pose3D botPose = LimeUtil.getResult().getBotpose();

		double boxX = botPose.getPosition().x;
		double botY = botPose.getPosition().y;

		Pose visionPose = new Pose(boxX, botY, 0 - follower.getPose().getHeading(),
				FTCCoordinates.INSTANCE)
				.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

		DebugUtil.logAdd("[BOT POSE]");
		DebugUtil.logAdd("X (in): " + boxX);
		DebugUtil.logAdd("Y (in): " + botY);
		DebugUtil.logAdd("HEADING: " + botPose.getOrientation());
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("[VISION POSE]");
		DebugUtil.logAdd("X (in): " + visionPose.getX());
		DebugUtil.logAdd("Y (in): " + visionPose.getY());
		DebugUtil.logAdd("HEADING: " + Math.toDegrees(visionPose.getHeading()));
		DebugUtil.logAdd(" ");

		DebugUtil.logAdd("[ODOMETRY]");
		DebugUtil.logAdd("X (in): " + pose.getX());
		DebugUtil.logAdd("Y (in): " + pose.getY());
		DebugUtil.logAdd("HEADING: " + Math.toDegrees(pose.getHeading()));

		DebugUtil.logAdd(" ");

		DebugUtil.update();
	}
}
