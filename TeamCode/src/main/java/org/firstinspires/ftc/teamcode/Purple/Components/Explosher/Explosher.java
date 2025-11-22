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
	public static final double CLOSE_SWEET = 0.45;
	public static final double FAR_SWEET = 1.0;

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

		this.motor = new MotorConfig.Builder(hardwareMap, Names.EXPLOSHER, MotorConfig.Position.EXPLOSHER, 28, 6000).build();

		this.finger = hardwareMap.get(Servo.class, fingerConfig.getName());
		this.fingerConfig = fingerConfig;

		stop();
		setDistanceState(DistanceState.NEAR);
	}

	/**
	 * Sets the target RPM for the shooter motor
	 *
	 * @param rpm The target RPM to set
	 */
	public void setRPM (double rpm)
	{

		this.targetRPM = rpm;
		motor.setTargetRPM(rpm);
	}

	/**
	 * Gets the current RPM of the shooter motor
	 *
	 * @return Current RPM value
	 */
	public double getCurrentRPM ()
	{

		return motor.getCurrentRPM();
	}

	/**
	 * Gets the target RPM of the shooter motor
	 *
	 * @return Target RPM value
	 */
	public double getTargetRPM ()
	{

		return targetRPM;
	}

	/**
	 * Stops the shooter motor
	 */
	public void stop ()
	{

		motor.stop();
		targetRPM = 0;
	}

	/**
	 * Gets the current finger servo state
	 *
	 * @return Current servo state
	 */
	public ServoConfig.ServoState getFingerState ()
	{

		return fingerConfig.getState();
	}

	/**
	 * Sets the finger servo state
	 *
	 * @param state The servo state to set
	 */
	public void setFingerState (ServoConfig.ServoState state)
	{

		fingerConfig.setState(state);

		if (state == ServoConfig.ServoState.ON)
			finger.setPosition(fingerConfig.getMaxPosition());
		else
			finger.setPosition(fingerConfig.getMinPosition());
	}

	/**
	 * Gets the current finger servo position
	 *
	 * @return Current finger position
	 */
	public double getFingerPosition ()
	{

		return finger.getPosition();
	}

	/**
	 * Sets the finger servo to a specific position
	 *
	 * @param position The position to set (clamped to valid range)
	 */
	public void setFingerPosition (double position)
	{

		finger.setPosition(fingerConfig.clamp(position));
	}

	/**
	 * Gets the current distance state
	 *
	 * @return Current distance state
	 */
	public DistanceState getDistanceState ()
	{

		return distanceState;
	}

	/**
	 * Sets the distance state and adjusts finger position accordingly
	 *
	 * @param state The distance state to set
	 */
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

	/**
	 * Cycles to the next distance state
	 */
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

	/**
	 * Updates the explosher subsystem - call in main loop
	 */
	public void update ()
	{

		motor.update();

		DebugUtil.logAdd("Explosher RPM: " +
				String.format("%.2f", getCurrentRPM()) +
				" / " +
				String.format("%.2f", getTargetRPM())
		);

		if (distanceState == DistanceState.AUTO)
			autoFingerAdjust();
	}

	/**
	 * Gets the maximum RPM capability of the shooter motor
	 *
	 * @return Maximum RPM value
	 */
	public double getMaxRPM ()
	{

		return motor.getMaxRPM();
	}

	public enum DistanceState
	{
		NEAR, MID, FAR, AUTO;

		/**
		 * Gets the next distance state in sequence
		 *
		 * @return Next distance state
		 */
		public DistanceState next ()
		{

			return values()[(ordinal() + 1) % values().length];
		}
	}
}