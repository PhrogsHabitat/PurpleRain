package org.firstinspires.ftc.teamcode.Purple.Components.Explosher;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Explosher
{

	public static final double CLOSE_SWEET = 0.45;
	public static final double FAR_SWEET = 1.0;
	public static final double RPM_SMOOTHING_ALPHA = 0.2;
	private final MotorConfig motor;
	private final MotorConfig motor2;
	private final MotorConfig exploRingMotor;
	private final Servo finger;
	private final ServoConfig fingerConfig;
	public boolean shouldRegress = false;
	public double dist;
	public double regressionSlope;
	public double regressionIntercept;
	public double smoothedTargetRPM = 0;
	private FingerState fingerState = FingerState.STOP;
	private double debugFingerPosition = Constants.FINGER_STOP_POSITION;
	private double targetRPM = 0;

	public Explosher (HardwareMap hardwareMap, ServoConfig fingerConfig)
	{
		// Main shooter motors with velocity control
		this.motor = new MotorConfig.Builder(hardwareMap, Names.EXPLOSHER, MotorConfig.Position.EXPLOSHER, 28, 6000).build();
		this.motor2 = new MotorConfig.Builder(hardwareMap, Names.EXPLOSHER_2, MotorConfig.Position.EXPLOSHER, 28, 6000).build();

		this.exploRingMotor = new MotorConfig.Builder(hardwareMap, Names.EXPLORING, MotorConfig.Position.EXPLOSHER, 1538, 435).setPositionCoefficient(0.05).setPositionTolerance(10).disableVelocityControl().build();

		this.finger = hardwareMap.get(Servo.class, fingerConfig.getName());
		this.fingerConfig = fingerConfig;

		// Stop everything and zero
		stop();
		setFingerState(FingerState.STOP);

		// Reset turret encoder to zero at startup
		resetRingPosition();

		calculateRegression();
	}

	/**
	 * Updates the explosher subsystem - call in main loop
	 */
	public void update ()
	{

		motor.update();
		motor2.update();
		exploRingMotor.update();  // Handles position control updates

		if (shouldRegress)
		{
			dist = LimeUtil.getTargetDistance();
			double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;

			smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
			smoothedTargetRPM = Math.max(0, Math.min(smoothedTargetRPM, motor.getMaxRPM()));
		}

		DebugUtil.logAdd("Explosher RPM: " +
				String.format("%.2f", getCurrentRPM()) +
				" / " +
				String.format("%.2f", getTargetRPM())
		);

		DebugUtil.logAdd("Finger State: " + fingerState +
				" | Pos: " + String.format("%.3f", getFingerPosition()));

		DebugUtil.logAdd("RING Pos: " + getRingPosition() +
				" | At Target: " + isRingAtTarget());
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
		motor2.setTargetRPM(rpm);
	}

	/**
	 * Sets raw power to the turret ring motor (for manual control)
	 *
	 * @param power Power value between -1.0 and 1.0
	 */
	public void setRingPower (double power)
	{

		exploRingMotor.setPower(power);
	}

	/**
	 * Sets the turret to a specific position (in encoder ticks)
	 * Uses position control with the specified power
	 *
	 * @param position Target position in encoder ticks
	 * @param power    Power to apply (0.0 to 1.0)
	 */
	public void setRingPosition (int position, double power)
	{

		exploRingMotor.runToPosition(position, power);
	}

	/**
	 * Gets the current turret position in encoder ticks
	 *
	 * @return Current turret position
	 */
	public int getRingPosition ()
	{

		return exploRingMotor.getCurrentPosition();
	}

	/**
	 * Checks if the turret is at its target position
	 *
	 * @return true if within tolerance of target
	 */
	public boolean isRingAtTarget ()
	{

		return exploRingMotor.atTargetPosition();
	}

	/**
	 * Resets the turret encoder position to zero
	 */
	public void resetRingPosition ()
	{

		exploRingMotor.resetEncoder();
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
	 * Stops the shooter motor and turret
	 */
	public void stop ()
	{

		motor.stop();
		motor2.stop();
		exploRingMotor.setPower(0);
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

		if (fingerState != FingerState.DEBUG)
		{
			return;
		}

		debugFingerPosition += increment;
		debugFingerPosition = fingerConfig.clamp(debugFingerPosition);
		setFingerPosition(debugFingerPosition);

		DebugUtil.logAdd("Debug Finger Pos: " + String.format("%.3f", debugFingerPosition));
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

	private void calculateRegression ()
	{

		double[][] calibrationPoints = {
				{59, 2900},
				{65, 2850},
				{77, 3000},
				{80, 3200},
				{94, 3100}
		};

		int n = calibrationPoints.length;
		double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

		for (double[] point : calibrationPoints)
		{
			double distance = point[0];
			double rpm = point[1];
			sumX += distance;
			sumY += rpm;
			sumXY += distance * rpm;
			sumX2 += distance * distance;
		}

		regressionSlope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
		regressionIntercept = (sumY - regressionSlope * sumX) / n;
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