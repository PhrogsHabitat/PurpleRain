package org.firstinspires.ftc.teamcode.Purple.Extras;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Controls;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Utils.AutoRotateController;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleAuto", group = "Purple")
public class TeleAuto extends PurpleOpMode
{
	public static Pose startingPose;

	private Controls driver1;
	private Controls driver2;
	private Follower follower;
	private Explosher explosher;
	private AutoRotateController autoRotateController;
	private Vaccum vaccum;

	@Override
	public void create ()
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
		follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
		follower.update();
		follower.startTeleopDrive(true);

		PurpleMemory.initialize(hardwareMap, follower);
		LimeUtil.start(hardwareMap, 60);
		LimeUtil.setPipeline(0);

		autoRotateController = new AutoRotateController();
		autoRotateController.registerWithPanels();

		explosher = new Explosher(hardwareMap);
		explosher.setAimPoint(autoRotateController.getAimX(), autoRotateController.getAimY());
		vaccum = new Vaccum(hardwareMap);

		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update ()
	{
		driver1.update();
		driver2.update();
		follower.update();
		LimeUtil.update();
		PurpleMemory.Instance.update();

		explosher.setAimPoint(autoRotateController.getAimX(), autoRotateController.getAimY());
		updateDrive();
		updateShooter();
		updateIntake();

		explosher.update();
		vaccum.update();

		DebugUtil.logAdd("TeleAuto Pose: " + follower.getPose());
		DebugUtil.logAdd("TeleAuto Aim Error: " + String.format("%.2f", autoRotateController.getLastHeadingErrorDeg()));
		DebugUtil.logAdd("TeleAuto Aim Turn: " + String.format("%.3f", autoRotateController.getLastTurnPower()));
		DebugUtil.logAdd("TeleAuto RPM: " + String.format("%.1f", explosher.getCurrentRPM()));
		DebugUtil.logAdd("TeleAuto Gate: " + explosher.getGateState());
		DebugUtil.logAdd("TeleAuto Intake: " + String.format("%.2f", vaccum.getPower()));
		DebugUtil.update();
	}

	private void updateDrive ()
	{
		double forward = -gamepad1.left_stick_y;
		double strafe = -gamepad1.left_stick_x;
		double turn = -gamepad1.right_stick_x;

		if (driver1.isPressed("right_bumper"))
		{
			turn = autoRotateController.updateTurn(follower.getPose());
		}
		else
		{
			autoRotateController.reset();
		}

		follower.setTeleOpDrive(forward, strafe, turn, true);
	}

	private void updateShooter ()
	{
		double shooterStick = driver2.getLeftStickY();
		if (shooterStick > Constants.JOYSTICK_DEADZONE)
		{
			explosher.setRegressionEnabled(true);
			explosher.setRPM(explosher.hasRegressionTarget() ? explosher.getSmoothedTargetRPM() : 2800.0);
		}
		else if (shooterStick < -Constants.JOYSTICK_DEADZONE)
		{
			explosher.setRPM(-4000);
		}
		else
		{
			explosher.stop();
		}

		if (driver2.justPressed("left_trigger"))
		{
			explosher.toggleGateState();
		}
	}

	private void updateIntake ()
	{
		if (driver2.isPressed("y"))
		{
			vaccum.setPower(Vaccum.DEFAULT_POW);
		}
		else if (driver2.isPressed("x"))
		{
			vaccum.setPower(-Vaccum.DEFAULT_POW);
		}
		else
		{
			vaccum.stop();
		}
	}

	@Override
	public void destroy ()
	{
		explosher.stop();
		vaccum.stop();
	}
}
