package org.firstinspires.ftc.teamcode.Purple.Utils;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class AutoRotateController
{
	public double aimX = 128.0;
	public double aimY = 130.0;
	public double shooterOffsetDeg = 0.0;

	public double pidKp = 0.018;
	public double pidKi = 0.0;
	public double pidKd = 0.0;
	public double pidDeadbandDeg = 0.5;
	public double pidMaxTurn = 0.45;
	public double pidMaxSlew = 6.0;
	public double pidIntegralLimit = 35.0;
	public double pidDerivativeAlpha = 0.2;
	public double pidFlipErrorDeg = 180.0;
	public double pidFlipTurn = 0.45;
	public double pidFlipSlew = 14.0;

	@IgnoreConfigurable
	private double lastTurnPower = 0.0;
	@IgnoreConfigurable
	private double lastHeadingErrorDeg = 0.0;
	@IgnoreConfigurable
	private double lastTargetHeadingDeg = 0.0;
	@IgnoreConfigurable
	private double pidIntegral = 0.0;
	@IgnoreConfigurable
	private double pidPreviousError = 0.0;
	@IgnoreConfigurable
	private double pidDerivative = 0.0;
	@IgnoreConfigurable
	private long pidLastTimestampNs = 0L;

	public void registerWithPanels ()
	{
		PanelsConfigurables.INSTANCE.refreshClass(this);
	}

	public double updateTurn (Pose pose)
	{
		if (pose == null)
		{
			reset();
			return 0.0;
		}

		double targetHeadingDeg = getTargetHeadingDeg(pose);
		double headingErrorDeg = normalizeDegrees(targetHeadingDeg - Math.toDegrees(pose.getHeading()));
		double dt = getDtSeconds();

		lastTargetHeadingDeg = targetHeadingDeg;
		lastHeadingErrorDeg = headingErrorDeg;

		if (Math.abs(headingErrorDeg) < pidDeadbandDeg)
		{
			pidIntegral = 0.0;
			pidDerivative = 0.0;
			pidPreviousError = headingErrorDeg;
			lastTurnPower = slew(lastTurnPower, 0.0, dt, pidMaxSlew);
			return lastTurnPower;
		}

		if (Math.abs(headingErrorDeg) >= pidFlipErrorDeg)
		{
			pidIntegral = 0.0;
			pidPreviousError = headingErrorDeg;
			lastTurnPower = slew(lastTurnPower, Math.copySign(pidFlipTurn, headingErrorDeg), dt, pidFlipSlew);
			return lastTurnPower;
		}

		pidIntegral += headingErrorDeg * dt;
		pidIntegral = MathUtil.clamp(pidIntegral, -pidIntegralLimit, pidIntegralLimit);

		double rawDerivative = (headingErrorDeg - pidPreviousError) / dt;
		pidDerivative += pidDerivativeAlpha * (rawDerivative - pidDerivative);

		double requestedTurn =
				(pidKp * headingErrorDeg) +
				(pidKi * pidIntegral) +
				(pidKd * pidDerivative);
		requestedTurn = MathUtil.clamp(requestedTurn, -pidMaxTurn, pidMaxTurn);

		lastTurnPower = slew(lastTurnPower, requestedTurn, dt, pidMaxSlew);
		pidPreviousError = headingErrorDeg;
		return lastTurnPower;
	}

	public void reset ()
	{
		lastTurnPower = 0.0;
		lastHeadingErrorDeg = 0.0;
		lastTargetHeadingDeg = 0.0;
		pidIntegral = 0.0;
		pidPreviousError = 0.0;
		pidDerivative = 0.0;
		pidLastTimestampNs = 0L;
	}

	public double getAimX ()
	{
		return aimX;
	}

	public double getAimY ()
	{
		return aimY;
	}

	public double getLastTurnPower ()
	{
		return lastTurnPower;
	}

	public double getLastHeadingErrorDeg ()
	{
		return lastHeadingErrorDeg;
	}

	public double getLastTargetHeadingDeg ()
	{
		return lastTargetHeadingDeg;
	}

	private double getTargetHeadingDeg (Pose pose)
	{
		double deltaX = aimX - pose.getX();
		double deltaY = aimY - pose.getY();
		return normalizeDegrees(Math.toDegrees(Math.atan2(deltaY, deltaX)) + shooterOffsetDeg);
	}

	private double getDtSeconds ()
	{
		long nowNs = System.nanoTime();
		double dt = pidLastTimestampNs == 0L ? 0.02 : (nowNs - pidLastTimestampNs) / 1_000_000_000.0;
		pidLastTimestampNs = nowNs;
		return MathUtil.clamp(dt, 0.001, 0.1);
	}

	private double slew (double currentValue, double targetValue, double dt, double maxSlew)
	{
		double maxStep = Math.abs(maxSlew) * dt;
		return MathUtil.clamp(targetValue, currentValue - maxStep, currentValue + maxStep);
	}

	private double normalizeDegrees (double degrees)
	{
		while (degrees > 180.0)
		{
			degrees -= 360.0;
		}

		while (degrees < -180.0)
		{
			degrees += 360.0;
		}

		return degrees;
	}
}
