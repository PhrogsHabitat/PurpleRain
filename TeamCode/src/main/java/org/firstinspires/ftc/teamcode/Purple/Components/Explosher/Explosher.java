package org.firstinspires.ftc.teamcode.Purple.Components.Explosher;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

public class Explosher
{
	public static final double CLOSE_SWEET = 0.45;
	public static final double FAR_SWEET = 1.0;

	private final MotorConfig motor;
	private final Servo finger;
	private final ServoConfig fingerConfig;
	private FingerState fingerState = FingerState.STOP;
	private double debugFingerPosition = Constants.FINGER_STOP_POSITION;
	private double targetRPM = 0;

	public Explosher (HardwareMap hardwareMap, ServoConfig fingerConfig)
	{
		this.motor = new MotorConfig.Builder(hardwareMap, Names.EXPLOSHER, MotorConfig.Position.EXPLOSHER, 28, 6000).build();
		this.finger = hardwareMap.get(Servo.class, fingerConfig.getName());
		this.fingerConfig = fingerConfig;

		stop();
		setFingerState(FingerState.STOP);
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
	 * Gets the current finger state
	 *
	 * @return Current finger state
	 */
	public FingerState getFingerStateEnum ()
	{
		return fingerState;
	}

	/**
	 * Sets the finger state and adjusts finger position accordingly
	 *
	 * @param state The finger state to set
	 */
	public void setFingerState (FingerState state)
	{
		this.fingerState = state;
		fingerConfig.setState(ServoConfig.ServoState.ON); // Always keep servo powered

		switch (state)
		{
			case STOP:
				setFingerPosition(Constants.FINGER_STOP_POSITION);
				break;
			case PASS:
				setFingerPosition(Constants.FINGER_PASS_POSITION);
				break;
			case DEBUG:
				// In debug mode, keep current debug position
				setFingerPosition(debugFingerPosition);
				break;
		}
	}

	/**
	 * Cycles to the next finger state (STOP -> PASS -> DEBUG -> STOP)
	 */
	public void cycleFingerState ()
	{
		setFingerState(fingerState.next());
	}

	/**
	 * Adjusts the debug finger position by a small increment
	 *
	 * @param increment Positive to increase, negative to decrease
	 */
	public void adjustDebugFingerPosition (double increment)
	{
		if (fingerState != FingerState.DEBUG) {
			return;
		}

		debugFingerPosition += increment;
		debugFingerPosition = fingerConfig.clamp(debugFingerPosition);
		setFingerPosition(debugFingerPosition);

		DebugUtil.logAdd("Debug Finger Pos: " + String.format("%.3f", debugFingerPosition));
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

		DebugUtil.logAdd("Finger State: " + fingerState +
				" | Pos: " + String.format("%.3f", getFingerPosition()));
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

	public enum FingerState
	{
		STOP, PASS, DEBUG;

		/**
		 * Gets the next finger state in sequence
		 *
		 * @return Next finger state
		 */
		public FingerState next ()
		{
			return values()[(ordinal() + 1) % values().length];
		}
	}
}