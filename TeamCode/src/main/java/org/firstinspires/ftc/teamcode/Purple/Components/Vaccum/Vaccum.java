package org.firstinspires.ftc.teamcode.Purple.Components.Vaccum;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Vaccum
{
	public static final double DEFAULT_POW = 1.0;

	public final MotorConfig intakeMotor;
	public final MotorConfig midtakeMotor;
	private double currentPower = 0;

	/**
	 * Creates a new Vaccum subsystem
	 *
	 * @param hardwareMap The robot's hardware map
	 */
	public Vaccum (HardwareMap hardwareMap)
	{

		intakeMotor = new MotorConfig.Builder(hardwareMap, Names.INTAKE, MotorConfig.Position.INTAKE).disableVelocityControl().build();
		midtakeMotor = new MotorConfig.Builder(hardwareMap, Names.INTAKE, MotorConfig.Position.MIDTAKE).disableVelocityControl().build();

		stop();
	}

	/**
	 * Gets the current intake power level
	 *
	 * @return Current power value (-1 to 1)
	 */
	public double getPower ()
	{
		return currentPower;
	}

	/**
	 * Sets the intake power level
	 *
	 * @param power Power value between -1 and 1
	 */
	public void setPower (double power)
	{
		currentPower = power;
		intakeMotor.setPower(power);
	}

	/**
	 * Runs only midtake at negative power
	 */
	public void swagReverse(double power)
	{
		currentPower = power;
//		midtakeMotor.setPower((power));
	}

	/**
	 * Stops the intake motors
	 */
	public void stop ()
	{
		setPower(0);
	}

	/**
	 * Gets the current RPM of the intake motor
	 *
	 * @return Intake motor RPM
	 */
	public double getIntakeCurrentRPM ()
	{

		return intakeMotor.getCurrentRPM();
	}

	/**
	 * Gets the current RPM of the midtake motor
	 *
	 * @return Midtake motor RPM
	 */
	public double getMidtakeCurrentRPM ()
	{

		return 0.0;
	}

	/**
	 * Updates the vaccum subsystem - call in main loop
	 */
	public void update ()
	{

//		midtakeMotor.update();
		intakeMotor.update();

		DebugUtil.logAdd("Intake RPM: " +
				String.format("%.1f", getIntakeCurrentRPM()));

		DebugUtil.logAdd("Midtake RPM: " +
				String.format("%.1f", getMidtakeCurrentRPM()));
	}
}