package org.firstinspires.ftc.teamcode.Purple.Components.Vaccum;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Vaccum
{
	public static final double DEFAULT_POW = 1.0;
	private static final double FINGER_TOLERANCE = 0.01;  // position tolerance for state transitions

	public final MotorConfig intakeMotor;

	// Three distinct finger servos – assumes Names.FINGER1, FINGER2, FINGER3 are defined
	private final Servo[] fingers;
	private final ServoConfig fingerConfig;

	// State machine for each finger
	private FingerFlickState[] fingerStates;
	private double[] fingerTargetPositions;  // only used for debugging/logging

	private double currentPower = 0;

	/**
	 * Creates a new Vaccum subsystem
	 *
	 * @param hardwareMap The robot's hardware map
	 */
	public Vaccum(HardwareMap hardwareMap)
	{
		intakeMotor = new MotorConfig.Builder(hardwareMap, Names.INTAKE, MotorConfig.Position.INTAKE)
				.disableVelocityControl()
				.build();

		this.fingerConfig = Constants.FINGER_SERVO_CONFIG;

		// Initialize the three finger servos
		fingers = new Servo[3];
		fingers[0] = hardwareMap.get(Servo.class, Names.FINGER_1);
		fingers[1] = hardwareMap.get(Servo.class, Names.FINGER_2);
		fingers[2] = hardwareMap.get(Servo.class, Names.FINGER_3);

		// Initialize state trackers
		fingerStates = new FingerFlickState[3];
		fingerTargetPositions = new double[3];
		for (int i = 0; i < 3; i++)
		{
			fingerStates[i] = FingerFlickState.IDLE;
			fingerTargetPositions[i] = fingerConfig.getMinPosition();
			// Set initial position to minimum (rest)
			fingers[i].setPosition(fingerConfig.getMinPosition());
		}

		stop();
	}

	// ==================== INTAKE MOTOR ====================

	public double getPower()
	{
		return currentPower;
	}

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

	// ==================== FINGER FLICK STATE MACHINE ====================

	/**
	 * Triggers a flick for the specified finger.
	 * The finger will move to max position, then immediately back to min.
	 *
	 * @param fingerIndex 0, 1, or 2
	 */
	public void flickFinger(int fingerIndex)
	{
		if (fingerIndex < 0 || fingerIndex >= fingers.length) return;

		// Start the flick sequence – move to max position
		fingerStates[fingerIndex] = FingerFlickState.FLICKING_UP;
		fingers[fingerIndex].setPosition(fingerConfig.getMaxPosition());
		fingerTargetPositions[fingerIndex] = fingerConfig.getMaxPosition();
	}

	/**
	 * Alternative method that accepts the Servo object.
	 * Finds which index the servo belongs to and flicks it.
	 */
	public void flickFinger(Servo finger)
	{
		for (int i = 0; i < fingers.length; i++)
		{
			if (fingers[i] == finger)
			{
				flickFinger(i);
				return;
			}
		}
	}

	/**
	 * Gets the current flick state of a finger.
	 */
	public FingerFlickState getFingerState(int index)
	{
		return fingerStates[index];
	}

	/**
	 * Gets the current servo position of a finger.
	 */
	public double getFingerPosition(int index)
	{
		return fingers[index].getPosition();
	}

	// ==================== UPDATE LOOP ====================

	/**
	 * Updates the vaccum subsystem – call in main loop.
	 * Handles state transitions for finger flicking.
	 */
	public void update()
	{
		intakeMotor.update();

		// Process each finger's state machine
		for (int i = 0; i < fingers.length; i++)
		{
			double currentPos = fingers[i].getPosition();

			switch (fingerStates[i])
			{
				case FLICKING_UP:
					// Wait until the servo reaches the max position
					if (Math.abs(currentPos - fingerConfig.getMaxPosition()) <= FINGER_TOLERANCE)
					{
						// Transition to moving down
						fingerStates[i] = FingerFlickState.FLICKING_DOWN;
						fingers[i].setPosition(fingerConfig.getMinPosition());
						fingerTargetPositions[i] = fingerConfig.getMinPosition();
					}
					break;

				case FLICKING_DOWN:
					// Wait until the servo returns to the min position
					if (Math.abs(currentPos - fingerConfig.getMinPosition()) <= FINGER_TOLERANCE)
					{
						// Flick complete – return to idle
						fingerStates[i] = FingerFlickState.IDLE;
					}
					break;

				case IDLE:
				default:
					// Nothing to do
					break;
			}
		}

		// Debug logging (optional)
		DebugUtil.logAdd("Intake RPM: " + String.format("%.1f", getIntakeCurrentRPM()));
		for (int i = 0; i < fingers.length; i++)
		{
			DebugUtil.logAdd("Finger" + i + " State: " + fingerStates[i] +
					" Pos: " + String.format("%.3f", fingers[i].getPosition()));
		}
	}

	// ==================== INTERNAL ENUM ====================

	/**
	 * States for the finger flick state machine.
	 */
	public enum FingerFlickState
	{
		IDLE,
		FLICKING_UP,
		FLICKING_DOWN
	}
}