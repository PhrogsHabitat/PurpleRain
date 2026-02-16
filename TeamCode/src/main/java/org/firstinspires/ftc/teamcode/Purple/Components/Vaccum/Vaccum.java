package org.firstinspires.ftc.teamcode.Purple.Components.Vaccum;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Vaccum
{
	private static final long FINGER_HOLD_MS = 500;

	public static final double DEFAULT_POW = 1.0;
	public final MotorConfig intakeMotor;

	private final ServoConfig finger1;
	private final ServoConfig finger2;
	private final ServoConfig finger3;

	private enum FingerState { READY, HOLDING_MAX, HOLDING_MIN }
	private final FingerState[] fingerStates = new FingerState[]{
			FingerState.READY,
			FingerState.READY,
			FingerState.READY
	};
	private final long[] fingerStateStartTimesMs = new long[3];

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

		finger1.setPosition(finger1.getMinPosition());
		finger2.setPosition(finger2.getMinPosition());
		finger3.setPosition(finger3.getMinPosition());

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
		startFlick(0);
	}

	public void flickFinger2()
	{
		startFlick(1);
	}

	public void flickFinger3()
	{
		startFlick(2);
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
		long nowMs = System.currentTimeMillis();

		updateFingerState(0, nowMs);
		updateFingerState(1, nowMs);
		updateFingerState(2, nowMs);

		DebugUtil.logAdd("Finger positions: " + finger1.getPosition() + ", " + finger2.getPosition() + ", " + finger3.getPosition());
	}

	private boolean startFlick(int index)
	{
		if (!isValidFingerIndex(index) || fingerStates[index] != FingerState.READY)
		{
			return false;
		}

		ServoConfig finger = getFinger(index);
		if (finger == null)
		{
			return false;
		}

		finger.setPosition(finger.getMaxPosition());
		fingerStates[index] = FingerState.HOLDING_MAX;
		fingerStateStartTimesMs[index] = System.currentTimeMillis();
		return true;
	}

	private void updateFingerState(int index, long nowMs)
	{
		if (!isValidFingerIndex(index))
		{
			return;
		}

		ServoConfig finger = getFinger(index);
		if (finger == null)
		{
			return;
		}

		if (fingerStates[index] == FingerState.HOLDING_MAX)
		{
			if (nowMs - fingerStateStartTimesMs[index] >= FINGER_HOLD_MS)
			{
				finger.setPosition(finger.getMinPosition());
				fingerStates[index] = FingerState.HOLDING_MIN;
				fingerStateStartTimesMs[index] = nowMs;
			}
			return;
		}

		if (fingerStates[index] == FingerState.HOLDING_MIN &&
				nowMs - fingerStateStartTimesMs[index] >= FINGER_HOLD_MS)
		{
			fingerStates[index] = FingerState.READY;
		}
	}

	private boolean isValidFingerIndex(int index)
	{
		return index >= 0 && index < 3;
	}

	private ServoConfig getFinger(int index)
	{
		switch (index)
		{
			case 0: return finger1;
			case 1: return finger2;
			case 2: return finger3;
			default: return null;
		}
	}
}
