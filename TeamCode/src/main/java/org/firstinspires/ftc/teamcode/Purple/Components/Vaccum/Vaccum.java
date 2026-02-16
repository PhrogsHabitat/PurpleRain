package org.firstinspires.ftc.teamcode.Purple.Components.Vaccum;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Vaccum
{
	public static final double DEFAULT_POW = 1.0;
	private static final double SERVO_POSITION_EPSILON = 0.001;
	public final MotorConfig intakeMotor;

	private final ServoConfig finger1;
	private final ServoConfig finger2;
	private final ServoConfig finger3;

	private enum FingerState { IDLE, FLICKING }
	private FingerState finger1State = FingerState.IDLE;
	private FingerState finger2State = FingerState.IDLE;
	private FingerState finger3State = FingerState.IDLE;

	private double currentPower = 0;

	public Vaccum(HardwareMap hardwareMap)
	{
		intakeMotor = new MotorConfig.Builder(hardwareMap, Names.INTAKE, MotorConfig.Position.INTAKE)
				.disableVelocityControl()
				.build();

		finger1 = new ServoConfig.Builder(hardwareMap, Names.FINGER_1)
				.setRange(Constants.FINGER_MIN, Constants.FINGER_MAX)
				.reversed()
				.build();
		finger2 = new ServoConfig.Builder(hardwareMap, Names.FINGER_2)
				.setRange(Constants.FINGER_MIN, Constants.FINGER_MAX)
				.reversed()
				.build();
		finger3 = new ServoConfig.Builder(hardwareMap, Names.FINGER_3)
				.setRange(Constants.FINGER_MIN, Constants.FINGER_MAX)
				.reversed()
				.build();

		stop();
	}

	public double getPower() { return currentPower; }

	public void setPower(double power)
	{
		currentPower = power;
		intakeMotor.setPower(power);
	}

	public void stop()
	{
		setPower(0);
	}

	public double getIntakeCurrentRPM()
	{
		return intakeMotor.getCurrentRPM();
	}

	public double getFingerPosition(int index)
	{
		switch (index)
		{
			case 0: return finger1.getPosition();
			case 1: return finger2.getPosition();
			case 2: return finger3.getPosition();
			default: return 0;
		}
	}

	public void adjustFingerPosition(int index, double delta)
	{
		switch (index)
		{
			case 0: finger1.setPosition(finger1.getPosition() + delta); break;
			case 1: finger2.setPosition(finger2.getPosition() + delta); break;
			case 2: finger3.setPosition(finger3.getPosition() + delta); break;
		}
	}

	public void flickFinger1()
	{
		finger1.setPosition(finger1.getMaxPosition());
	}

	public void flickFinger2()
	{
		finger2.setPosition(finger2.getMaxPosition());
	}

	public void flickFinger3()
	{
		finger3.setPosition(finger3.getMaxPosition());
	}

	public void flickFinger(int index)
	{
		switch (index)
		{
			case 0: flickFinger1(); break;
			case 1: flickFinger2(); break;
			case 2: flickFinger3(); break;
		}
	}

	public void flickFingers()
	{
		flickFinger1();
		flickFinger2();
		flickFinger3();
	}

	public void update()
	{
		intakeMotor.update();

		if (finger1.getPosition() == finger1.getMaxPosition())
		{
			finger1.setPosition(finger1.getMinPosition());
		}

		if (finger2.getPosition() == finger2.getMaxPosition())
		{
			finger2.setPosition(finger2.getMinPosition());
		}

		if (finger3.getPosition() == finger3.getMaxPosition())
		{
			finger3.setPosition(finger3.getMinPosition());
		}

		DebugUtil.logAdd("Finger positions: " + finger1.getPosition() + ", " + finger2.getPosition() + ", " + finger3.getPosition());
	}
}
