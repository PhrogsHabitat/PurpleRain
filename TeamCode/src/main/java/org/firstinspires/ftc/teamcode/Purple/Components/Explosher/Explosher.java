package org.firstinspires.ftc.teamcode.Purple.Components.Explosher;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Position;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

public class Explosher
{
	public static final double CLOSE_SWEET = 0.45;
	public static final double FAR_SWEET = 1.0;
	public static final double RPM_SMOOTHING_ALPHA = 0.2;
	public static final double HOOD_SMOOTHING_ALPHA = 0.2;

	private static final double[][] RPM_CALIBRATION_POINTS = {
			{101.8, 2400},
			{112, 2500},
			{78, 2400},
			{147, 3100},
			{57, 1700}
	};

	private static final double[][] HOOD_CALIBRATION_POINTS = {
			{101.8, 0.48},
			{112, 0.53},
			{78, 1.0},
			{147, 1.0},
			{57, 0.45}
	};

	private static final double EXPLORE_TICKS_PER_DEG = 1200.0 / 180.0;
	private static final double DEFAULT_EXPLORE_DEG_POW = 1.0;
	private static final double PID_KP = 0.036; // 36
	private static final double PID_KI = 0.0015;
	private static final double PID_KD = 0.0020;
	private static final double PID_DEAD = 0.1;
	private static final double PID_MAX_POW = 1.0;
	private static final double PID_MAX_SLEW = 6.0;
	private static final double PID_INT_LIM = 90.0;
	private static final double PID_DER_A = 0.2;
	private static final double PID_FLIP_ERR = 80.0;
	private static final double PID_FLIP_POW = 1.0;
	private static final double PID_FLIP_SLEW = 14.0;
	private static final double AIM_WARMUP_S = 0.20;
	private static final double AIM_TARGET_ALPHA = 0.24;
	private static final double AIM_TARGET_NOISE_DEG = 0.45;
	private static final double MIN_DEG = -180.0;
	private static final double MAX_DEG = 180.0;
	private static final double AIM_ZERO_FROM_FRONT_DEG = 180.0;
	private static final double AIM_BEARING_SIGN = -1.0;
	private static final double AIM_X = 128;
	private static final double AIM_Y = 130;

	private final MotorConfig motor;
	private final MotorConfig motor2;
	private final MotorConfig exploringMotor;
	private final ServoConfig fingerConfig;

	private FingerState fingerState = FingerState.STOP;
	private boolean regressionEnabled = false;
	private boolean hasRegressionTarget = false;
	private double regressionSlope;
	private double regressionIntercept;
	private double hoodRegressionSlope;
	private double hoodRegressionIntercept;
	private double smoothedTargetRPM = 0.0;
	private double smoothedTargetHoodPosition = Constants.FINGER_STOP_POSITION;
	private double debugFingerPosition = Constants.FINGER_STOP_POSITION;
	private double targetRPM = 0.0;
	private double aimX = AIM_X;
	private double aimY = AIM_Y;
	private double aimPow = 0.0;
	private double aimErr = 0.0;
	private double pidInt = 0.0;
	private double pidErr = 0.0;
	private double pidDer = 0.0;
	private double pidPow = 0.0;
	private long pidNs = 0;
	private long aimWarmupNs = 0;
	private double filteredAimTargetDeg = 0.0;
	private boolean hasFilteredAimTarget = false;
	private int exploringTargetPos = 0;

	public Explosher (HardwareMap hardwareMap)
	{

		motor = new MotorConfig.Builder(
				hardwareMap,
				Names.EXPLOSHER,
				MotorConfig.Position.EXPLOSHER,
				28,
				6000
		).inverted().build();

		motor2 = new MotorConfig.Builder(
				hardwareMap,
				Names.EXPLOSHER_2,
				MotorConfig.Position.EXPLOSHER,
				28,
				6000
		).inverted().build();

		exploringMotor = new MotorConfig.Builder(
				hardwareMap,
				Names.EXPLORING,
				MotorConfig.Position.EXPLOSHER,
				1538,
				435
		).setPositionCoefficient(0.05)
				.setPositionTolerance(10)
				.disableVelocityControl()
				.build();

		fingerConfig = new ServoConfig.Builder(hardwareMap, Names.HOOD)
				.setRange(Constants.HOOD_MIN, Constants.HOOD_MAX)
				.build();

		calculateRegression();
		stop();
		setFingerState(FingerState.STOP);
		smoothedTargetHoodPosition = getFingerPosition();
		resetExploringPos();
	}

	/**
	 * Updates the shooter, ring angle, and regression targets.
	 */
	public void update ()
	{

		motor.update();
		motor2.update();
		exploringMotor.update();

		if (regressionEnabled)
		{
			Double targetDistance = getRegressionDistance();
			hasRegressionTarget = targetDistance != null;
			if (hasRegressionTarget)
			{
				double rawTargetRPM = (regressionSlope * targetDistance) + regressionIntercept;
				smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
				smoothedTargetRPM = MathUtil.clamp(smoothedTargetRPM, 0.0, motor.getMaxRPM());

				double rawTargetHoodPosition = (hoodRegressionSlope * targetDistance) + hoodRegressionIntercept;
				smoothedTargetHoodPosition += HOOD_SMOOTHING_ALPHA * (rawTargetHoodPosition - smoothedTargetHoodPosition);
				smoothedTargetHoodPosition = fingerConfig.clamp(smoothedTargetHoodPosition);
				if (fingerState != FingerState.DEBUG)
				{
					setFingerPosition(smoothedTargetHoodPosition);
				}
			}
		}
		else
		{
			hasRegressionTarget = false;
		}

		DebugUtil.logAdd("Explosher RPM: " +
				String.format("%.2f", getCurrentRPM()) +
				" / " +
				String.format("%.2f", getTargetRPM()));
		DebugUtil.logAdd("Finger State: " + fingerState +
				" | Pos: " + String.format("%.3f", getFingerPosition()) +
				" | Auto: " + String.format("%.3f", smoothedTargetHoodPosition));
		DebugUtil.logAdd("Exploring Pos: " + getExploringPos() +
				" | At Target: " + isExploringAtPos());
	}

	/**
	 * Stops shooter and exploring motors and clears target RPM.
	 */
	public void stop ()
	{

		motor.stop();
		motor2.stop();
		setExploringPow(0.0);
		targetRPM = 0.0;
	}

	/**
	 * Gets whether automatic regression updates are enabled.
	 *
	 * @return True if regression is enabled.
	 */
	public boolean isRegressionEnabled ()
	{

		return regressionEnabled;
	}

	/**
	 * Enables or disables distance-based regression updates.
	 *
	 * @param enabled True to update smoothed RPM and hood targets from odometry distance.
	 */
	public void setRegressionEnabled (boolean enabled)
	{

		if (regressionEnabled == enabled)
		{
			return;
		}

		regressionEnabled = enabled;
		if (enabled)
		{
			smoothedTargetRPM = getCurrentRPM();
			smoothedTargetHoodPosition = getFingerPosition();
		}
	}

	/**
	 * Gets the smoothed RPM target generated by regression.
	 *
	 * @return Current smoothed regression RPM.
	 */
	public double getSmoothedTargetRPM ()
	{

		return smoothedTargetRPM;
	}

	/**
	 * Gets the smoothed hood position target generated by regression.
	 *
	 * @return Current smoothed regression hood target.
	 */
	public double getSmoothedTargetHoodPosition ()
	{

		return smoothedTargetHoodPosition;
	}

	/**
	 * Gets whether odometry produced a valid regression distance this cycle.
	 *
	 * @return True if the regression target is valid.
	 */
	public boolean hasRegressionTarget ()
	{

		return hasRegressionTarget;
	}

	/**
	 * Gets odometry-based distance from robot pose to the Explosher aim point.
	 *
	 * @return Distance in inches, or null when pose is unavailable.
	 */
	public Double getDistanceToTarget ()
	{

		return getRegressionDistance();
	}

	/**
	 * Sets shooter target RPM on both flywheel motors.
	 *
	 * @param rpm Target RPM.
	 */
	public void setRPM (double rpm)
	{

		targetRPM = rpm;
		motor.setTargetRPM(rpm);
		motor2.setTargetRPM(rpm);
	}

	/**
	 * Gets the current shooter RPM.
	 *
	 * @return Current RPM.
	 */
	public double getCurrentRPM ()
	{

		return motor.getCurrentRPM();
	}

	/**
	 * Gets the current shooter target RPM.
	 *
	 * @return Target RPM.
	 */
	public double getTargetRPM ()
	{

		return targetRPM;
	}

	/**
	 * Gets the maximum supported shooter RPM.
	 *
	 * @return Shooter max RPM.
	 */
	public double getMaxRPM ()
	{

		return motor.getMaxRPM();
	}

	/**
	 * Updates auto-aim output from robot pose and applies PID power.
	 *
	 * @param pose Current robot pose.
	 */
	public void updateAim (Pose pose)
	{

		if (pose == null)
		{
			resetAimState();
			setExploringPow(0.0);
			return;
		}

		long nowNs = System.nanoTime();
		if (aimWarmupNs == 0)
		{
			aimWarmupNs = nowNs;
		}

		double targetDeg = filterAimTargetDeg(getAimDeg(pose));
		exploringTargetPos = (int) Math.round(targetDeg * EXPLORE_TICKS_PER_DEG);
		double warmupSec = (nowNs - aimWarmupNs) / 1_000_000_000.0;
		if (warmupSec < AIM_WARMUP_S)
		{
			pidInt = 0.0;
			pidDer = 0.0;
			pidErr = targetDeg - getExploringDeg();
			pidPow = 0.0;
			pidNs = 0;
			aimErr = pidErr;
			setExploringPow(0.0);
			return;
		}

		setExploringPow(getPidPow(targetDeg));
	}

	/**
	 * Updates auto-aim output from PurpleMemory position and applies PID power.
	 *
	 * @param position Current robot position from PurpleMemory.
	 */
	public void updateAim (Position position)
	{

		if (position == null)
		{
			updateAim((Pose) null);
			return;
		}

		Pose pose = new Pose(
				position.getX(),
				position.getY(),
				Math.toRadians(position.getHeading())
		);

		updateAim(pose);
	}

	/**
	 * Sets the world-space point used by the auto-aim solver.
	 *
	 * @param x Target X.
	 * @param y Target Y.
	 */
	public void setAimPoint (double x, double y)
	{

		aimX = x;
		aimY = y;
	}

	/**
	 * Gets the current auto-aim power command.
	 *
	 * @return Current PID output power.
	 */
	public double getAimPow ()
	{

		return aimPow;
	}

	/**
	 * Gets the current auto-aim error in degrees.
	 *
	 * @return Current angle error.
	 */
	public double getAimErr ()
	{

		return aimErr;
	}

	/**
	 * Sets raw power to the exploring motor.
	 *
	 * @param power Power in range [-1, 1].
	 */
	public void setExploringPow (double power)
	{

		aimPow = power;
		exploringMotor.setPower(power);
	}

	/**
	 * Sets exploring motor position in encoder ticks.
	 *
	 * @param position Target position in ticks.
	 * @param power    Run power in range [0, 1].
	 */
	public void setExploringPos (int position, double power)
	{

		exploringTargetPos = position;
		exploringMotor.runToPosition(position, power);
	}

	/**
	 * Sets exploring angle in degrees.
	 *
	 * @param angleDegrees Target angle in degrees.
	 * @param power        Run power in range [0, 1].
	 */
	public void setExploringDeg (double angleDegrees, double power)
	{

		int targetTicks = (int) Math.round(angleDegrees * EXPLORE_TICKS_PER_DEG);
		setExploringPos(targetTicks, power);
	}

	/**
	 * Gets exploring position in encoder ticks.
	 *
	 * @return Current position in ticks.
	 */
	public int getExploringPos ()
	{

		return exploringMotor.getCurrentPosition();
	}

	/**
	 * Gets exploring target position in encoder ticks.
	 *
	 * @return Target position in ticks.
	 */
	public int getExploringTargetPos ()
	{

		return exploringTargetPos;
	}

	/**
	 * Gets exploring angle in degrees.
	 *
	 * @return Current angle.
	 */
	public double getExploringDeg ()
	{

		return getExploringPos() / EXPLORE_TICKS_PER_DEG;
	}

	/**
	 * Sets exploring angle in degrees with default power.
	 *
	 * @param angleDegrees Target angle in degrees.
	 */
	public void setExploringDeg (double angleDegrees)
	{

		setExploringDeg(angleDegrees, DEFAULT_EXPLORE_DEG_POW);
	}

	/**
	 * Gets exploring target angle in degrees.
	 *
	 * @return Target angle in degrees.
	 */
	public double getExploringTargetDeg ()
	{

		return exploringTargetPos / EXPLORE_TICKS_PER_DEG;
	}

	/**
	 * Gets whether exploring motor is at its target position.
	 *
	 * @return True if at target.
	 */
	public boolean isExploringAtPos ()
	{

		return exploringMotor.atTargetPosition();
	}

	/**
	 * Resets exploring encoder to zero and clears aim integrators.
	 */
	public void resetExploringPos ()
	{

		exploringMotor.resetEncoder();
		exploringTargetPos = 0;
		resetAimState();
	}

	/**
	 * Gets current raw finger servo power state.
	 *
	 * @return Finger servo on/off state.
	 */
	public ServoConfig.ServoState getFingerState ()
	{

		return fingerConfig.getState();
	}

	/**
	 * Sets raw finger servo power state.
	 *
	 * @param state Servo on/off state.
	 */
	public void setFingerState (ServoConfig.ServoState state)
	{

		fingerConfig.setState(state);
		if (state == ServoConfig.ServoState.ON)
		{
			fingerConfig.setPosition(fingerConfig.getMaxPosition());
		}
		else
		{
			fingerConfig.setPosition(fingerConfig.getMinPosition());
		}
	}

	/**
	 * Sets logical finger behavior state.
	 *
	 * @param state Logical finger state.
	 */
	public void setFingerState (FingerState state)
	{

		fingerState = state;
		fingerConfig.setState(ServoConfig.ServoState.ON);

		switch (state)
		{
			case STOP:
				setFingerPosition(Constants.FINGER_STOP_POSITION);
				break;
			case PASS:
				setFingerPosition(Constants.FINGER_PASS_POSITION);
				break;
			case DEBUG:
				setFingerPosition(debugFingerPosition);
				break;
		}
	}

	/**
	 * Gets logical finger state.
	 *
	 * @return Current finger state.
	 */
	public FingerState getFingerStateEnum ()
	{

		return fingerState;
	}

	/**
	 * Cycles to the next finger state.
	 */
	public void cycleFingerState ()
	{

		setFingerState(fingerState.next());
	}

	/**
	 * Gets finger position.
	 *
	 * @return Current finger position.
	 */
	public double getFingerPosition ()
	{

		return fingerConfig.getPosition();
	}

	/**
	 * Sets finger position.
	 *
	 * @param position Target finger position.
	 */
	public void setFingerPosition (double position)
	{

		fingerConfig.setPosition(position);
	}

	/**
	 * Nudges debug finger position when in debug mode.
	 *
	 * @param increment Increment added to current debug position.
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

	private void calculateRegression ()
	{

		double[] rpmRegression = calculateLinearRegression(RPM_CALIBRATION_POINTS);
		regressionSlope = rpmRegression[0];
		regressionIntercept = rpmRegression[1];

		double[] hoodRegression = calculateLinearRegression(HOOD_CALIBRATION_POINTS);
		hoodRegressionSlope = hoodRegression[0];
		hoodRegressionIntercept = hoodRegression[1];
	}

	private double[] calculateLinearRegression (double[][] calibrationPoints)
	{

		int pointCount = calibrationPoints.length;
		if (pointCount < 2)
		{
			throw new IllegalArgumentException("Need at least 2 calibration points for regression.");
		}

		double sumX = 0.0;
		double sumY = 0.0;
		double sumXY = 0.0;
		double sumX2 = 0.0;

		for (double[] point : calibrationPoints)
		{
			double distance = point[0];
			double yValue = point[1];
			sumX += distance;
			sumY += yValue;
			sumXY += distance * yValue;
			sumX2 += distance * distance;
		}

		double denominator = pointCount * sumX2 - sumX * sumX;
		if (Math.abs(denominator) < 1e-9)
		{
			throw new IllegalArgumentException("Calibration points have invalid X distribution for regression.");
		}

		double slope = (pointCount * sumXY - sumX * sumY) / denominator;
		double intercept = (sumY - slope * sumX) / pointCount;
		return new double[]{slope, intercept};
	}

	/**
	 * Gets the distance from the robot's current odometry pose to the aim point.
	 * This method now uses ONLY odometry (PurpleMemory) – no LimeLight fallback.
	 *
	 * @return Distance in inches, or null if pose unavailable.
	 */
	private Double getRegressionDistance ()
	{

		if (PurpleMemory.Instance == null)
		{
			return null;
		}
		Pose pose = PurpleMemory.Instance.curPose();
		if (pose == null)
		{
			return null;
		}
		double deltaX = aimX - pose.getX();
		double deltaY = aimY - pose.getY();
		return Math.hypot(deltaX, deltaY);
	}

	private double getAimDeg (Pose pose)
	{

		double robotX = pose.getX();
		double robotY = pose.getY();
		double deltaY = aimY - robotY;
		double deltaX = aimX - robotX;
		double robotHeadingDeg = Math.toDegrees(pose.getHeading());
		double bearingDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));
		double relativeBearingDeg = normDeg(bearingDeg - robotHeadingDeg);
		double targetDeg = normDeg(AIM_ZERO_FROM_FRONT_DEG + (AIM_BEARING_SIGN * relativeBearingDeg));

		DebugUtil.logAdd("DX: " + deltaX);
		DebugUtil.logAdd("DY: " + deltaY);
		DebugUtil.logAdd("Angle: " + bearingDeg);
		DebugUtil.logAdd("RelAngle: " + relativeBearingDeg);
		DebugUtil.logAdd("TargetDeg: " + targetDeg);

		return MathUtil.clamp(targetDeg, MIN_DEG, MAX_DEG);
	}

	private double getPidPow (double targetDeg)
	{

		double currentDeg = getExploringDeg();
		double clampedTargetDeg = MathUtil.clamp(targetDeg, MIN_DEG, MAX_DEG);
		double error = clampedTargetDeg - currentDeg;
		long nowNs = System.nanoTime();
		double dt = pidNs == 0 ? 0.02 : (nowNs - pidNs) / 1_000_000_000.0;

		pidNs = nowNs;
		dt = MathUtil.clamp(dt, 0.001, 0.1);

		if (Math.abs(error) < 0.01)
		{
			pidInt = 0.0;
			pidDer = 0.0;
			pidErr = error;
			pidPow = slewPow(pidPow, 0.0, dt, PID_MAX_SLEW);
			aimErr = error;
			return pidPow;
		}

		if (Math.abs(error) >= PID_FLIP_ERR)
		{
			pidInt = 0.0;

			double boostPow = Math.copySign(PID_FLIP_POW, error);
			boostPow = hardStop(currentDeg, boostPow);
			pidPow = slewPow(pidPow, boostPow, dt, PID_FLIP_SLEW);
			pidErr = error;
			aimErr = error;
			return pidPow;
		}

		pidInt += error * dt;
		pidInt = MathUtil.clamp(pidInt, -PID_INT_LIM, PID_INT_LIM);

		double rawDer = (error - pidErr) / dt;
		pidDer += PID_DER_A * (rawDer - pidDer);

		double targetPow = (PID_KP * error) + (PID_KI * pidInt) + (PID_KD * pidDer);
		targetPow = MathUtil.clamp(targetPow, -PID_MAX_POW, PID_MAX_POW);
		targetPow = hardStop(currentDeg, targetPow);

		pidPow = slewPow(pidPow, targetPow, dt, PID_MAX_SLEW);
		pidErr = error;
		aimErr = error;
		return pidPow;
	}

	private double filterAimTargetDeg (double rawTargetDeg)
	{

		double clampedRawDeg = MathUtil.clamp(rawTargetDeg, MIN_DEG, MAX_DEG);
		if (!hasFilteredAimTarget)
		{
			filteredAimTargetDeg = clampedRawDeg;
			hasFilteredAimTarget = true;
			return filteredAimTargetDeg;
		}

		double delta = normDeg(clampedRawDeg - filteredAimTargetDeg);
		if (Math.abs(delta) <= AIM_TARGET_NOISE_DEG)
		{
			return filteredAimTargetDeg;
		}

		filteredAimTargetDeg = normDeg(filteredAimTargetDeg + (AIM_TARGET_ALPHA * delta));
		return filteredAimTargetDeg;
	}

	private double slewPow (double currentPow, double targetPow, double dt, double maxSlew)
	{

		double maxStep = maxSlew * dt;
		return MathUtil.clamp(targetPow, currentPow - maxStep, currentPow + maxStep);
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

	private double hardStop (double currentDeg, double requestedPow)
	{

		if (currentDeg >= MAX_DEG && requestedPow > 0)
		{
			return 0;
		}

		if (currentDeg <= MIN_DEG && requestedPow < 0)
		{
			return 0;
		}

		return requestedPow;
	}

	private void resetAimState ()
	{

		aimErr = 0.0;
		pidInt = 0.0;
		pidErr = 0.0;
		pidDer = 0.0;
		pidPow = 0.0;
		pidNs = 0;
		aimWarmupNs = 0;
		filteredAimTargetDeg = 0.0;
		hasFilteredAimTarget = false;
	}

	public enum FingerState
	{
		STOP,
		PASS,
		DEBUG;

		/**
		 * Gets the next finger state in sequence.
		 *
		 * @return Next finger state.
		 */
		public FingerState next ()
		{

			return values()[(ordinal() + 1) % values().length];
		}
	}
}