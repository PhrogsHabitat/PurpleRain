package org.firstinspires.ftc.teamcode.Purple.Components.Vaccum;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Vaccum
{
	public static final double DEFAULT_POW = 1.0;

	private final MotorConfig intakeMotor;

	private final MotorConfig midtakeMotor;
	private double currentPower = 0.0;

	public Vaccum (HardwareMap hardwareMap)
	{
		intakeMotor = new MotorConfig.Builder(hardwareMap, Names.INTAKE, MotorConfig.Position.INTAKE)
				.disableVelocityControl()
				.build();

		stop();
		midtakeMotor = new MotorConfig.Builder(hardwareMap, Names.MIDITAKE, MotorConfig.Position.MIDTAKE)
				.disableVelocityControl()
				.build();

		stop();
	}

	/**
	 * Updates the intake motor state.
	 */
	public void update ()
	{
		intakeMotor.update();
		midtakeMotor.update();
		DebugUtil.logAdd("Intake Power: " + String.format("%.2f", currentPower));
	}

	/**
	 * Sets intake motor power.
	 *
	 * @param power Intake power from -1.0 to 1.0.
	 */
	public void setPower (double power)
	{
		currentPower = Math.max(-1.0, Math.min(1.0, power));
		intakeMotor.setPower(currentPower);
		midtakeMotor.setPower(currentPower);
	}

	public double getPower ()
	{
		return currentPower;
	}

	public void stop ()
	{
		setPower(0.0);
	}

	public double getIntakeCurrentRPM ()
	{
		return intakeMotor.getCurrentRPM();
	}
}
