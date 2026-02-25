package org.firstinspires.ftc.teamcode.Purple.Components.Explosher;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
	public static final double RPM_SMOOTHING_ALPHA = 0.2;
	private static final double EXPLORE_TICKS_PER_DEG = 1200.0 / 180.0;
	private static final double DEFAULT_EXPLORE_DEG_POW = 1;
	private static final double PID_KP = 0.036;
	private static final double PID_KI = 0.0012;
	private static final double PID_KD = 0.0020;
	private static final double PID_DEAD = 0.3;
	private static final double PID_MAX_POW = 1.0;
	private static final double PID_MAX_SLEW = 6.0;
	private static final double PID_INT_LIM = 35.0;
	private static final double PID_DER_A = 0.2;
	private static final double PID_FLIP_ERR = 80.0;
	private static final double PID_FLIP_POW = 1.0;
	private static final double PID_FLIP_SLEW = 14.0;
	private static final double MIN_DEG = -180.0;
	private static final double MAX_DEG = 180.0;
	private static final double AIM_ZERO_FROM_FRONT_DEG = 180.0;
	private static final double AIM_BEARING_SIGN = -1.0;
	private static final double AIM_X = 60;
	private static final double AIM_Y = 0;
	private final MotorConfig motor;
	private final MotorConfig motor2;
	private final MotorConfig exploringMotor;
	private final ServoConfig fingerConfig;
	public boolean shouldRegress = false;
	public double dist;
	public double regressionSlope;
	public double regressionIntercept;
	public double smoothedTargetRPM = 0;
	private FingerState fingerState = FingerState.STOP;
	private double debugFingerPosition = Constants.FINGER_STOP_POSITION;
	private double targetRPM = 0;
	private double aimX = AIM_X;
	private double aimY = AIM_Y;
	private double aimPow = 0;
	private double aimErr = 0;
	private double pidInt = 0;
	private double pidErr = 0;
	private double pidDer = 0;
	private double pidPow = 0;
	private long pidNs = 0;

	public Explosher (HardwareMap hardwareMap)
	{

		this(
				hardwareMap,
				new ServoConfig.Builder(hardwareMap, Names.HOOD)
						.setRange(Constants.HOOD_MIN, Constants.HOOD_MAX)
						.build()
		);
	}

	public Explosher (HardwareMap hardwareMap, ServoConfig fingerConfig)
	{
		// Main shooter motors with velocity control
		this.motor = new MotorConfig.Builder(hardwareMap, Names.EXPLOSHER, MotorConfig.Position.EXPLOSHER, 28, 6000).build();
		this.motor2 = new MotorConfig.Builder(hardwareMap, Names.EXPLOSHER_2, MotorConfig.Position.EXPLOSHER, 28, 6000).build();

		this.exploringMotor = new MotorConfig.Builder(hardwareMap, Names.EXPLORING, MotorConfig.Position.EXPLOSHER, 1538, 435).setPositionCoefficient(0.05).setPositionTolerance(10).disableVelocityControl().build();

		this.fingerConfig = fingerConfig;

		// Stop everything and zero
		stop();
		setFingerState(FingerState.STOP);

		// Reset exploring encoder to zero at startup
		resetExploringPos();

		calculateRegression();
	}

	/**
	 * Updates the explosher subsystem - call in main loop
	 */
	public void update ()
	{

		motor.update();
		motor2.update();
		exploringMotor.update(); // Handles position control updates

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

		DebugUtil.logAdd("Exploring Pos: " + getExploringPos() +
				" | At Target: " + isExploringAtPos());
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
	 * Runs the exploring auto-aim pipeline using robot pose only.
	 *
	 * @param pose Current robot pose
	 */
	public void updateAim (Pose pose)
	{

		if (pose == null)
		{
			resetAimState();
			setExploringPow(0);
			return;
		}

		double targetDeg = getAimDeg(pose);
		setExploringPow(getPidPow(targetDeg));
	}

	/**
	 * Sets raw power to the exploring motor.
	 *
	 * @param power Power value between -1.0 and 1.0
	 */
	public void setExploringPow (double power)
	{

		aimPow = power;
		exploringMotor.setPower(power);
	}

	/**
	 * Sets the exploring motor to a specific position (in encoder ticks)
	 * Uses position control with the specified power
	 *
	 * @param position Target position in encoder ticks
	 * @param power    Power to apply (0.0 to 1.0)
	 */
	public void setExploringPos (int position, double power)
	{

		exploringMotor.runToPosition(position, power);
	}

	/**
	 * Sets exploring to a specific angle in degrees using position control.
	 * Conversion ratio: 1200 ticks == 180 degrees.
	 *
	 * @param angleDegrees Target angle in degrees
	 * @param power        Power to apply (0.0 to 1.0)
	 */
	public void setExploringDeg (double angleDegrees, double power)
	{

		int targetTicks = (int) Math.round(angleDegrees * EXPLORE_TICKS_PER_DEG);
		setExploringPos(targetTicks, power);
	}

	/**
	 * Gets the current exploring position in encoder ticks
	 *
	 * @return Current exploring position
	 */
	public int getExploringPos ()
	{

		return exploringMotor.getCurrentPosition();
	}

	/**
	 * Gets the current exploring angle in degrees.
	 * Conversion ratio: 1200 ticks == 180 degrees.
	 *
	 * @return Current exploring angle in degrees
	 */
	public double getExploringDeg ()
	{

		return getExploringPos() / EXPLORE_TICKS_PER_DEG;
	}

	/**
	 * Sets exploring to a specific angle in degrees using default power.
	 *
	 * @param angleDegrees Target angle in degrees
	 */
	public void setExploringDeg (double angleDegrees)
	{

		setExploringDeg(angleDegrees, DEFAULT_EXPLORE_DEG_POW);
	}

	/**
	 * Checks if exploring is at its target position
	 *
	 * @return true if within tolerance of target
	 */
	public boolean isExploringAtPos ()
	{

		return exploringMotor.atTargetPosition();
	}

	/**
	 * Resets the exploring encoder position to zero
	 */
	public void resetExploringPos ()
	{

		exploringMotor.resetEncoder();
		resetAimState();
	}

	public void setAimPoint (double x, double y)
	{

		aimX = x;
		aimY = y;
	}

	public double getAimPow ()
	{

		return aimPow;
	}

	public double getAimErr ()
	{

		return aimErr;
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
	 * Stops the shooter motor and exploring motor
	 */
	public void stop ()
	{

		motor.stop();
		motor2.stop();
		setExploringPow(0);
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
			fingerConfig.setPosition(fingerConfig.getMaxPosition());
		else
			fingerConfig.setPosition(fingerConfig.getMinPosition());
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

		return fingerConfig.getPosition();
	}

	/**
	 * Sets the finger servo to a specific position
	 *
	 * @param position The position to set (clamped to valid range)
	 */
	public void setFingerPosition (double position)
	{

		fingerConfig.setPosition(position);
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

	// Backward-compatible aliases
	public void setRingPower (double power)
	{

		setExploringPow(power);
	}

	public void setRingPosition (int position, double power)
	{

		setExploringPos(position, power);
	}

	public void setAngle (double angleDegrees, double power)
	{

		setExploringDeg(angleDegrees, power);
	}

	public int getRingPosition ()
	{

		return getExploringPos();
	}

	public double getAngle ()
	{

		return getExploringDeg();
	}

	public void setAngle (double angleDegrees)
	{

		setExploringDeg(angleDegrees);
	}

	public boolean isRingAtTarget ()
	{

		return isExploringAtPos();
	}

	public void resetRingPosition ()
	{

		resetExploringPos();
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

	private double getAimDeg (Pose pose)
	{

		double botX = pose.getX();
		double botY = pose.getY();

		double dY = aimY - botY;
		double dX = aimX - botX;

		double botHeadDeg = Math.toDegrees(pose.getHeading());
		double bearingDeg = Math.toDegrees(Math.atan2(dY, dX));
		double relBearingDeg = normDeg(bearingDeg - botHeadDeg);
		double targetDeg = normDeg(AIM_ZERO_FROM_FRONT_DEG + (AIM_BEARING_SIGN * relBearingDeg));

		DebugUtil.logAdd("DX: " + dX);
		DebugUtil.logAdd("DY: " + dY);
		DebugUtil.logAdd("Angle: " + bearingDeg);
		DebugUtil.logAdd("RelAngle: " + relBearingDeg);
		DebugUtil.logAdd("TargetDeg: " + targetDeg);

		return MathUtil.clamp(targetDeg, MIN_DEG, MAX_DEG);
	}

	private double getPidPow (double targetDeg)
	{

		double curDeg = getExploringDeg();
		double goalDeg = MathUtil.clamp(targetDeg, MIN_DEG, MAX_DEG);
		double err = goalDeg - curDeg;

		long nowNs = System.nanoTime();
		double dt = pidNs == 0 ? 0.02 : (nowNs - pidNs) / 1_000_000_000.0;
		pidNs = nowNs;
		dt = MathUtil.clamp(dt, 0.001, 0.1);

		if (Math.abs(err) < PID_DEAD)
		{
			pidInt = 0;
			pidDer = 0;
			pidErr = err;
			pidPow = slewPow(pidPow, 0, dt);
			aimErr = err;
			return pidPow;
		}

		if (Math.abs(err) >= PID_FLIP_ERR)
		{
			pidInt = 0;
			double boostPow = Math.copySign(PID_FLIP_POW, err);
			boostPow = hardStop(curDeg, boostPow);
			pidPow = slewPow(pidPow, boostPow, dt, PID_FLIP_SLEW);
			pidErr = err;
			aimErr = err;
			return pidPow;
		}

		pidInt += err * dt;
		pidInt = MathUtil.clamp(pidInt, -PID_INT_LIM, PID_INT_LIM);

		double rawDer = (err - pidErr) / dt;
		pidDer += PID_DER_A * (rawDer - pidDer);

		double targetPow = (PID_KP * err) +
				(PID_KI * pidInt) +
				(PID_KD * pidDer);
		targetPow = MathUtil.clamp(targetPow, -PID_MAX_POW, PID_MAX_POW);
		targetPow = hardStop(curDeg, targetPow);

		pidPow = slewPow(pidPow, targetPow, dt);
		pidErr = err;
		aimErr = err;
		return pidPow;
	}

	private double slewPow (double curPow, double targetPow, double dt)
	{

		return slewPow(curPow, targetPow, dt, PID_MAX_SLEW);
	}

	private double slewPow (double curPow, double targetPow, double dt, double maxSlew)
	{

		double maxStep = maxSlew * dt;
		return MathUtil.clamp(targetPow, curPow - maxStep, curPow + maxStep);
	}

	private double normDeg (double deg)
	{

		while (deg > 180)
		{
			deg -= 360;
		}

		while (deg < -180)
		{
			deg += 360;
		}

		return deg;
	}

	private double hardStop (double curDeg, double reqPow)
	{

		if (curDeg >= MAX_DEG && reqPow > 0)
		{
			return 0;
		}

		if (curDeg <= MIN_DEG && reqPow < 0)
		{
			return 0;
		}

		return reqPow;
	}

	private void resetAimState ()
	{

		aimErr = 0;
		pidInt = 0;
		pidErr = 0;
		pidDer = 0;
		pidPow = 0;
		pidNs = 0;
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
