package org.firstinspires.ftc.teamcode.Purple.Extras;

import org.firstinspires.ftc.teamcode.Purple.Components.Explosher.Explosher;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.OpMode.PurpleOpMode;
import org.firstinspires.ftc.teamcode.Purple.Components.Vaccum.Vaccum;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Controls;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TempOp", group = "Purple")
public class TempOp extends PurpleOpMode
{
	private Controls driver1;
	private Controls driver2;
	private MotorConfig fl;
	private MotorConfig fr;
	private MotorConfig bl;
	private MotorConfig br;
	private Explosher explosher;
	private Vaccum vaccum;

	@Override
	public void create ()
	{
		driver1 = new Controls(gamepad1);
		driver2 = new Controls(gamepad2);

		fl = new MotorConfig.Builder(hardwareMap, Names.FRONTLEFT, MotorConfig.Position.FRONT_LEFT, 2150.76, 312)
				.disableVelocityControl().build();
		fr = new MotorConfig.Builder(hardwareMap, Names.FRONTRIGHT, MotorConfig.Position.FRONT_RIGHT, 2150.76, 312)
				.disableVelocityControl().build();
		bl = new MotorConfig.Builder(hardwareMap, Names.BACKLEFT, MotorConfig.Position.BACK_LEFT, 2150.76, 312)
				.disableVelocityControl().build();
		br = new MotorConfig.Builder(hardwareMap, Names.BACKRIGHT, MotorConfig.Position.BACK_RIGHT, 2150.76, 312)
				.disableVelocityControl().build();

		explosher = new Explosher(hardwareMap);
		vaccum = new Vaccum(hardwareMap);
		DebugUtil.setTelemetry(telemetry);
	}

	@Override
	public void update ()
	{
		driver1.update();
		driver2.update();

		updateDrive();
		updateShooter();
		updateIntake();

		explosher.update();
		vaccum.update();

		DebugUtil.logAdd("TempOp RPM: " + String.format("%.1f", explosher.getCurrentRPM()));
		DebugUtil.logAdd("TempOp Gate: " + explosher.getGateState());
		DebugUtil.logAdd("TempOp Intake: " + String.format("%.2f", vaccum.getPower()));
		DebugUtil.update();
	}

	private void updateDrive ()
	{
		double forward = driver1.getLeftStickY();
		double strafe = driver1.getLeftStickX();
		double turn = driver1.getRightStickX();
		double powerScale = driver1.isPressed("left_stick_button") ?
				Constants.DRIVE_POWER_BOOST : Constants.DRIVE_POWER_SCALE;

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
		fl.stop();
		fr.stop();
		bl.stop();
		br.stop();
		explosher.stop();
		vaccum.stop();
	}
}
