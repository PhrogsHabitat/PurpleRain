// package org.firstinspires.ftc.teamcode.Purple.Components.Explosher;

// import com.arcrobotics.ftclib.hardware.motors.Motor;
// import com.bylazar.configurables.annotations.Configurable;
// import com.pedropathing.geometry.Pose;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.hardware.Servo;

// import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
// import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
// import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
// import org.firstinspires.ftc.teamcode.Purple.Constants;
// import org.firstinspires.ftc.teamcode.Purple.Names;
// import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
// import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

// @Configurable
// public class Explosher
// {
	
// 	public static final double CLOSE_SWEET = 0.45;
// 	public static final double FAR_SWEET = 1.0;
// 	public static final double RPM_SMOOTHING_ALPHA = 0.2;
	
// 	public boolean shouldRegress = false;
// 	public double dist;
// 	public double regressionSlope;
// 	public double regressionIntercept;

// 	private final MotorConfig motor;
// 	private final Servo finger;
// 	private final ServoConfig fingerConfig;
// 	private FingerState fingerState = FingerState.STOP;
// 	private double debugFingerPosition = Constants.FINGER_STOP_POSITION;
// 	private double targetRPM = 0;
// 	public double smoothedTargetRPM = 0;
// 	public static double kP = 0.000005, kV = 0.00018, kS = 0.02;


// 	public enum FingerState
// 	{
// 		STOP, PASS, DEBUG;

// 		/**
// 		 * Gets the next finger state in sequence
// 		 *
// 		 * @return Next finger state
// 		 */
// 		public FingerState next ()
// 		{
// 			return values()[(ordinal() + 1) % values().length];
// 		}
// 	}

// 	public Explosher (HardwareMap hardwareMap, ServoConfig fingerConfig)
// 	{
// 		this.motor = new MotorConfig.Builder(hardwareMap, Names.EXPLOSHER, MotorConfig.Position.EXPLOSHER, 28, 6000).build();
// 		this.motor.setVelocityDirectionReversed(true);
// 		this.motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
// 		this.finger = hardwareMap.get(Servo.class, fingerConfig.getName());
// 		this.fingerConfig = fingerConfig;

// 		// Stop everything and zero
// 		// 30435

// 		stop();
// 		setFingerState(FingerState.STOP);

// 		calculateRegression();
// 	}

// 	/**
// 	 * Updates the explosher subsystem - call in main loop
// 	 */
// 	public void update ()
// 	{
// 		this.motor.setVelocityCoefficients(kP,0,0); // i: 45
// 		this.motor.setFeedforwardCoefficients(kS,kV,0);

// 		motor.update();

// 		if (shouldRegress)
// 		{
// 			dist = LimeUtil.getTargetDistance();
// 			double rawTargetRPM = (regressionSlope * dist) + regressionIntercept;

// 			smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
// 		    smoothedTargetRPM = Math.max(0, Math.min(smoothedTargetRPM, motor.getMaxRPM()));
// 		}

// 		DebugUtil.logAdd("Explosher RPM: " +
// 				String.format("%.2f", getCurrentRPM()) +
// 				" / " +
// 				String.format("%.2f", getTargetRPM())
// 		);

// 		DebugUtil.logAdd("Finger State: " + fingerState +
// 				" | Pos: " + String.format("%.3f", getFingerPosition()));
// 	}

// 	/**
// 	 * Sets the target RPM for the shooter motor
// 	 *
// 	 * @param rpm The target RPM to set
// 	 */
// 	public void setRPM (double rpm)
// 	{
// 		this.targetRPM = rpm;
// 		motor.setPower(calcPID(rpm, getCurrentRPM()));
// 	}

// 	/**
// 	 * Gets the current RPM of the shooter motor
// 	 *
// 	 * @return Current RPM value
// 	 */
// 	public double getCurrentRPM ()
// 	{
// 		return motor.getRPM();
// 	}

// 	/**
// 	 * Gets the target RPM of the shooter motor
// 	 *
// 	 * @return Target RPM value
// 	 */
// 	public double getTargetRPM ()
// 	{
// 		return targetRPM;
// 	}

// 	/**
// 	 * Stops the shooter motor
// 	 */
// 	public void stop ()
// 	{
// 		motor.stop();
// 		targetRPM = 0;
// 	}

// 	/**
// 	 * Gets the current finger servo state
// 	 *
// 	 * @return Current servo state
// 	 */
// 	public ServoConfig.ServoState getFingerState ()
// 	{
// 		return fingerConfig.getState();
// 	}

// 	/**
// 	 * Sets the finger servo state
// 	 *
// 	 * @param state The servo state to set
// 	 */
// 	public void setFingerState (ServoConfig.ServoState state)
// 	{
// 		fingerConfig.setState(state);
// 		if (state == ServoConfig.ServoState.ON)
// 			finger.setPosition(fingerConfig.getMaxPosition());
// 		else
// 			finger.setPosition(fingerConfig.getMinPosition());
// 	}

// 	/**
// 	 * Gets the current finger servo position
// 	 *
// 	 * @return Current finger position
// 	 */
// 	public double getFingerPosition ()
// 	{
// 		return finger.getPosition();
// 	}

// 	/**
// 	 * Sets the finger servo to a specific position
// 	 *
// 	 * @param position The position to set (clamped to valid range)
// 	 */
// 	public void setFingerPosition (double position)
// 	{
// 		finger.setPosition(fingerConfig.clamp(position));
// 	}

// 	/**
// 	 * Gets the current finger state
// 	 *
// 	 * @return Current finger state
// 	 */
// 	public FingerState getFingerStateEnum ()
// 	{
// 		return fingerState;
// 	}

// 	/**
// 	 * Sets the finger state and adjusts finger position accordingly
// 	 *
// 	 * @param state The finger state to set
// 	 */
// 	public void setFingerState (FingerState state)
// 	{
// 		this.fingerState = state;
// 		fingerConfig.setState(ServoConfig.ServoState.ON); // Always keep servo powered

// 		switch (state)
// 		{
// 			case STOP:
// 				setFingerPosition(Constants.FINGER_STOP_POSITION);
// 				break;
// 			case PASS:
// 				setFingerPosition(Constants.FINGER_PASS_POSITION);
// 				break;
// 			case DEBUG:
// 				// In debug mode, keep current debug position
// 				setFingerPosition(debugFingerPosition);
// 				break;
// 		}
// 	}

// 	/**
// 	 * Cycles to the next finger state (STOP -> PASS -> DEBUG -> STOP)
// 	 */
// 	public void cycleFingerState ()
// 	{
// 		setFingerState(fingerState.next());
// 	}

// 	/**
// 	 * Adjusts the debug finger position by a small increment
// 	 *
// 	 * @param increment Positive to increase, negative to decrease
// 	 */
// 	public void adjustDebugFingerPosition (double increment)
// 	{
// 		if (fingerState != FingerState.DEBUG) {
// 			return;
// 		}

// 		debugFingerPosition += increment;
// 		debugFingerPosition = fingerConfig.clamp(debugFingerPosition);
// 		setFingerPosition(debugFingerPosition);

// 		DebugUtil.logAdd("Debug Finger Pos: " + String.format("%.3f", debugFingerPosition));
// 	}
// 	/**
// 	 * Gets the maximum RPM capability of the shooter motor
// 	 *
// 	 * @return Maximum RPM value
// 	 */
// 	public double getMaxRPM ()
// 	{
// 		return motor.getMaxRPM();
// 	}

// 	private void calculateRegression ()
// 	{

// 		double[][] calibrationPoints = {
// 				{59, 2900},
// 				{65, 2850},
// 				{77, 3000},
// 				{80, 3200},
// 				{94, 3100}
// 		};

// 		int n = calibrationPoints.length;
// 		double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

// 		for (double[] point : calibrationPoints)
// 		{
// 			double distance = point[0];
// 			double rpm = point[1];
// 			sumX += distance;
// 			sumY += rpm;
// 			sumXY += distance * rpm;
// 			sumX2 += distance * distance;
// 		}

// 		regressionSlope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
// 		regressionIntercept = (sumY - regressionSlope * sumX) / n;
// 	}

// 	public double calcPID(double target, double current)
// 	{
// 		return kP * (target - current) + kV * target + kS;
// 	}
// }