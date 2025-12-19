package org.firstinspires.ftc.teamcode.Purple.Utils;

public class PathFollower
{
	private final PID xController;
	private final PID yController;
	private final PID headingController;
	private final double maxPower;
	private final double tolerance;

	private double targetX = 0;
	private double targetY = 0;
	private double targetHeading = 0;
	private boolean isFollowing = false;

	/**
	 * Constructs a PathFollower with PID gains, max power, and tolerances.
	 *
	 * @param kP               Proportional gain for PID.
	 * @param kI               Integral gain for PID.
	 * @param kD               Derivative gain for PID.
	 * @param maxPower         Maximum allowed power output.
	 * @param tolerance        Position tolerance for target.
	 * @param headingTolerance Heading tolerance for target.
	 */
	public PathFollower (double kP, double kI, double kD, double maxPower, double tolerance, double headingTolerance)
	{

		this.xController = new PID(kP, kI, kD);
		this.yController = new PID(kP, kI, kD);
		this.headingController = new PID(kP, kI, kD); // Tune heading separately if needed
		this.maxPower = maxPower;
		this.tolerance = tolerance;
	}

	/**
	 * Sets the target position and heading for the path follower.
	 *
	 * @param x       Target X position.
	 * @param y       Target Y position.
	 * @param heading Target heading in radians.
	 */
	public void setTarget (double x, double y, double heading)
	{

		this.targetX = x;
		this.targetY = y;
		this.targetHeading = heading;
		this.isFollowing = true;

		// Reset integrals when starting a new path
		xController.resetIntegral();
		yController.resetIntegral();
		headingController.resetIntegral();
	}

	/**
	 * Calculates the motor powers needed to follow the path to the target.
	 *
	 * @param currentX       Current X position.
	 * @param currentY       Current Y position.
	 * @param currentHeading Current heading in radians.
	 * @param targetX        Target X position.
	 * @param targetY        Target Y position.
	 * @param targetHeading  Target heading in radians.
	 * @return Array of motor powers: [fl, fr, bl, br].
	 */
	public double[] calculateMotorPowers (double currentX, double currentY, double currentHeading, double targetX, double targetY, double targetHeading)
	{

		if (!isFollowing)
		{
			setTarget(targetX, targetY, targetHeading);
		}

		// Calculate errors in the field coordinate system
		double xError = targetX - currentX;
		double yError = targetY - currentY;

		// Rotate errors to the robot's coordinate system based on its current heading[citation:6]
		double robotRelativeXError = xError * Math.cos(currentHeading) + yError * Math.sin(currentHeading);
		double robotRelativeYError = -xError * Math.sin(currentHeading) + yError * Math.cos(currentHeading);

		// Calculate heading error (shortest path)
		double headingError = normalizeAngle(targetHeading - currentHeading);

		// Calculate PID outputs
		double xPower = xController.calculate(robotRelativeXError);
		double yPower = yController.calculate(robotRelativeYError);
		double turnPower = headingController.calculate(headingError);

		// Clamp powers to maxPower
		xPower = clamp(xPower, -maxPower, maxPower);
		yPower = clamp(yPower, -maxPower, maxPower);
		turnPower = clamp(turnPower, -maxPower, maxPower);

		// Mecanum wheel power calculations[citation:6]
		double flPower = xPower + yPower + turnPower;
		double frPower = xPower - yPower - turnPower;
		double blPower = xPower - yPower + turnPower;
		double brPower = xPower + yPower - turnPower;

		// Normalize powers to maintain relative ratios while not exceeding max power
		double maxPowerVal = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
				Math.max(Math.abs(blPower), Math.abs(brPower)));
		if (maxPowerVal > 1.0)
		{
			flPower /= maxPowerVal;
			frPower /= maxPowerVal;
			blPower /= maxPowerVal;
			brPower /= maxPowerVal;
		}

		return new double[]{flPower, frPower, blPower, brPower};
	}

	/**
	 * Checks if the robot is at the target position and heading within tolerance.
	 *
	 * @param tolerance Allowed position tolerance.
	 * @return True if at target, false otherwise.
	 */
	public boolean isAtTarget (double tolerance)
	{

		if (!isFollowing) return false;
		// Check if both position and heading are within tolerance
		double distanceError = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY, 2));
		double headingError = Math.abs(normalizeAngle(targetHeading));
		return distanceError < tolerance && headingError < 0.1; // 0.1 rad ~5.7 degrees
	}

	/**
	 * Stops the path following process.
	 */
	public void stop ()
	{

		this.isFollowing = false;
	}

	/**
	 * Returns whether the path follower is currently following a path.
	 *
	 * @return True if following, false otherwise.
	 */
	public boolean isFollowing ()
	{

		return isFollowing;
	}

	private double normalizeAngle (double angle)
	{

		while (angle > Math.PI) angle -= 2 * Math.PI;
		while (angle < -Math.PI) angle += 2 * Math.PI;
		return angle;
	}

	private double clamp (double value, double min, double max)
	{

		return Math.max(min, Math.min(max, value));
	}

	// Simple nested PID class for internal use
	static class PID
	{
		private final double kP, kI, kD;
		private double integral = 0;
		private double lastError = 0;

		public PID (double kP, double kI, double kD)
		{

			this.kP = kP;
			this.kI = kI;
			this.kD = kD;
		}

		public double calculate (double error)
		{

			integral += error;
			double derivative = error - lastError;
			lastError = error;
			return kP * error + kI * integral + kD * derivative;
		}

		public void resetIntegral ()
		{

			integral = 0;
		}
	}
}