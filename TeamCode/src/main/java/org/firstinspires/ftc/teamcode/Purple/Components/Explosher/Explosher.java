package org.firstinspires.ftc.teamcode.Purple.Components.Explosher;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

@Configurable
public class Explosher
{
	public static double CLOSE_SWEET = 0.45;
	public static double FAR_SWEET = 1.0;
	public static double RPM_SMOOTHING_ALPHA = 0.2;
	public static double SHOOTER_KP = 0.0;
	public static double SHOOTER_KV = 0.000155;
	public static double SHOOTER_KS = 0.2;
	public static double RPM_STOP_DEADBAND = 1.0;

	public static double[][] RPM_CALIBRATION_POINTS = {
			{84, 4500},
			{45, 3700},
			{57, 4000},
			{93, 4400},
			{91, 4500},
			{116, 4900}

	};

	public static double AIM_X = 128;
	public static double AIM_Y = 130;

	private final MotorConfig motor;
	private final ServoConfig gateConfig;

	private GateState gateState = GateState.STOP;
	private boolean regressionEnabled = false;
	private boolean hasRegressionTarget = false;
	private double regressionSlope;
	private double regressionIntercept;
	private double smoothedTargetRPM = 0.0;
	private double targetRPM = 0.0;
	private double aimX = AIM_X;
	private double aimY = AIM_Y;

	public Explosher (HardwareMap hardwareMap)
	{
		motor = new MotorConfig.Builder(
				hardwareMap,
				Names.EXPLOSHER,
				MotorConfig.Position.EXPLOSHER,
				28,
				6000
		).zeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)
				.disableVelocityControl()
				.build();
		motor.setVelocityDirectionReversed(true);

		gateConfig = new ServoConfig.Builder(hardwareMap, Names.BLOCKER)
				.setRange(Constants.BLOCKER_MIN, Constants.BLOCKER_MAX)
				.build();
		gateConfig.setState(ServoConfig.ServoState.ON);

		calculateRegression();
		stop();
		setGateState(GateState.STOP);
	}

	/**
	 * Updates the shooter and distance-based RPM regression.
	 */
	public void update ()
	{
		updateShooterPower();
		motor.update();

		if (regressionEnabled)
		{
			Double targetDistance = getRegressionDistance();
			hasRegressionTarget = targetDistance != null;
			if (hasRegressionTarget)
			{
				double rawTargetRPM = (regressionSlope * targetDistance) + regressionIntercept;
				smoothedTargetRPM += RPM_SMOOTHING_ALPHA * (rawTargetRPM - smoothedTargetRPM);
				smoothedTargetRPM = MathUtil.clamp(smoothedTargetRPM, 0.0, motor.getMaxRPM());
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
		DebugUtil.logAdd("Gate State: " + gateState +
				" | Pos: " + String.format("%.3f", getGatePosition()));
	}

	/**
	 * Stops the shooter motor and clears the target RPM.
	 */
	public void stop ()
	{
		motor.stop();
		targetRPM = 0.0;
	}

	public boolean isRegressionEnabled ()
	{
		return regressionEnabled;
	}

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
		}
	}

	public double getSmoothedTargetRPM ()
	{
		return smoothedTargetRPM;
	}

	public boolean hasRegressionTarget ()
	{
		return hasRegressionTarget;
	}

	/**
	 * Gets odometry-based distance from robot pose to the aim point.
	 *
	 * @return Distance in inches, or null when pose is unavailable.
	 */
	public Double getDistanceToTarget ()
	{
		return getRegressionDistance();
	}

	/**
	 * Sets shooter target RPM.
	 *
	 * @param rpm Target RPM.
	 */
	public void setRPM (double rpm)
	{
		targetRPM = MathUtil.clamp(rpm, -motor.getMaxRPM(), motor.getMaxRPM());
	}

	public double getCurrentRPM ()
	{
		return motor.getCurrentRPM();
	}

	public double getTargetRPM ()
	{
		return targetRPM;
	}

	public double getMaxRPM ()
	{
		return motor.getMaxRPM();
	}

	/**
	 * Sets the field-space point used for RPM regression distance.
	 */
	public void setAimPoint (double x, double y)
	{
		aimX = x;
		aimY = y;
	}

	public GateState getGateState ()
	{
		return gateState;
	}

	public void setGateState (GateState state)
	{
		gateState = state;
		gateConfig.setState(ServoConfig.ServoState.ON);

		switch (state)
		{
			case STOP:
				setGatePosition(Constants.GATE_STOP_POSITION);
				break;
			case PASS:
				setGatePosition(Constants.GATE_PASS_POSITION);
				break;
		}
	}

	public void toggleGateState ()
	{
		setGateState(gateState == GateState.STOP ? GateState.PASS : GateState.STOP);
	}

	public double getGatePosition ()
	{
		return gateConfig.getPosition();
	}

	public void setGatePosition (double position)
	{
		gateConfig.setPosition(position);
	}

	private void calculateRegression ()
	{
		double[] rpmRegression = calculateLinearRegression(RPM_CALIBRATION_POINTS);
		regressionSlope = rpmRegression[0];
		regressionIntercept = rpmRegression[1];
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
			double rpm = point[1];
			sumX += distance;
			sumY += rpm;
			sumXY += distance * rpm;
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

	private void updateShooterPower ()
	{
		if (Math.abs(targetRPM) < RPM_STOP_DEADBAND)
		{
			motor.setPower(0.0);
			return;
		}

		double currentRPM = getCurrentRPM();
		double rpmError = targetRPM - currentRPM;
		double feedforwardPower = (SHOOTER_KV * targetRPM) + Math.copySign(SHOOTER_KS, targetRPM);
		double correctionPower = SHOOTER_KP * rpmError;
		double commandedPower = MathUtil.clamp(feedforwardPower + correctionPower, -1.0, 1.0);
		motor.setPower(commandedPower);
	}

	public enum GateState
	{
		STOP,
		PASS
	}
}
