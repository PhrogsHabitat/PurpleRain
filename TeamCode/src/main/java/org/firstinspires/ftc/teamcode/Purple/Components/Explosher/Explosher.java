package org.firstinspires.ftc.teamcode.Purple.Components.Explosher;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

public class Explosher
{

	// Shooter presets
	public static final double CLOSE_SWEET = 0.45;
	public static final double FAR_SWEET = 1.0;
	// Finger preset positions
	private static final double NEAR_POS = 0.0;
	private static final double MID_POS = 0.2;
	private static final double FAR_POS = 0.3;
	private final MotorConfig motor;
	private final Servo finger;
	private final ServoConfig fingerConfig;
	private DistanceState distanceState = DistanceState.NEAR;
	private double targetRPM = 0;

	public Explosher (HardwareMap hardwareMap, ServoConfig fingerConfig)
	{

		this.motor = new MotorConfig.Builder(
				hardwareMap,
				Names.EXPLOSHER,
				MotorConfig.Position.EXPLOSHER,
				28,
				6000
		)

				.build();

		this.finger = hardwareMap.get(Servo.class, fingerConfig.getName());
		this.fingerConfig = fingerConfig;

		setMotorState(MotorConfig.MotorState.OFF);
		setFingerState(ServoConfig.ServoState.OFF);
		setDistanceState(DistanceState.NEAR);
	}

	public void setRPM (double rpm)
	{
		this.targetRPM = rpm;
		motor.setTargetRPM(rpm);
	}

	// -------------------------------
	//   MOTOR / RPM CONTROL
	// -------------------------------

	public double getCurrentRPM ()
	{
		return motor.getCurrentRPM();
	}

	public double getTargetRPM ()
	{
		return targetRPM;
	}

	public MotorConfig.MotorState getMotorState ()
	{
		return motor.getState();
	}

	public void setMotorState (MotorConfig.MotorState state)
	{
		if (state == MotorConfig.MotorState.ON)
		{
			double rpmToUse = (targetRPM > 0) ? targetRPM : FAR_SWEET;
			motor.setTargetRPM(rpmToUse);
		} else
		{
			motor.stop();
			targetRPM = 0;
		}
	}

	public ServoConfig.ServoState getFingerState ()
	{
		return fingerConfig.getState();
	}

	// -------------------------------
	//     FINGER / SERVO CONTROL
	// -------------------------------

	public void setFingerState (ServoConfig.ServoState state)
	{
		fingerConfig.setState(state);

		if (state == ServoConfig.ServoState.ON)
			finger.setPosition(fingerConfig.getMaxPosition());
		else
			finger.setPosition(fingerConfig.getMinPosition());
	}

	public double getFingerPosition ()
	{
		return finger.getPosition();
	}

	public void setFingerPosition (double position)
	{
		finger.setPosition(fingerConfig.clamp(position));
	}

	public DistanceState getDistanceState ()
	{
		return distanceState;
	}

	// -------------------------------
	//     DISTANCE STATE LOGIC
	// -------------------------------

	public void setDistanceState (DistanceState state)
	{
		this.distanceState = state;

		switch (state)
		{
			case NEAR:
				setFingerPosition(NEAR_POS);
				break;

			case MID:
				setFingerPosition(MID_POS);
				break;

			case FAR:
				setFingerPosition(FAR_POS);
				break;

			case AUTO:
				autoFingerAdjust();
				break;
		}
	}

	public void cycleDistanceState ()
	{
		setDistanceState(distanceState.next());
	}

	private void autoFingerAdjust ()
	{
		if (distanceState != DistanceState.AUTO)
			return;

		double dist = LimeUtil.getTargetDistance();
		double clamped = MathUtil.clamp(dist, 0.0, 0.3);

		setFingerPosition(clamped);

		DebugUtil.logAdd("AutoFingerPos: " + clamped);
		DebugUtil.logAdd("Lime Best Target: " + LimeUtil.getResult());
		DebugUtil.logAdd("Lime TX: " + LimeUtil.getTx());
	}

	// -------------------------------
	//         AUTO MODE
	// -------------------------------

	public void update ()
	{
		if (distanceState == DistanceState.AUTO)
			autoFingerAdjust();

		DebugUtil.logAdd("Explosher RPM: " +
				String.format("%.2f", getCurrentRPM()) +
				" / " +
				String.format("%.2f", getTargetRPM())
		);
	}

	// -------------------------------
	//           UPDATE
	// -------------------------------

	public enum DistanceState
	{
		NEAR, MID, FAR, AUTO;

		public DistanceState next ()
		{
			return values()[(ordinal() + 1) % values().length];
		}
	}
}
