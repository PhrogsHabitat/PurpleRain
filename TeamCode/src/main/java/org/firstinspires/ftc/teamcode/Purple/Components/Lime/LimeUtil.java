package org.firstinspires.ftc.teamcode.Purple.Components.Lime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * FTC Limelight utility: wraps Limelight3A for easy, robust access
 * Consolidated version with static initialization and access methods
 */
public final class LimeUtil
{

	// Limelight instance
	private static Limelight3A limelight = null;
	public static IMU imu;

	private static boolean initialized = false;

	private LimeUtil ()
	{
		// Utility class - prevent instantiation
	}

	/**
	 * Initialize Limelight3A from hardwareMap
	 *
	 * @param hardwareMap FTC hardwareMap
	 * @param name        device name (e.g. "Limelight")
	 * @param pollHz      polling rate
	 * @return true if initialization successful
	 */
	public static boolean start (HardwareMap hardwareMap, String name, int pollHz)
	{

		try
		{
			limelight = hardwareMap.get(Limelight3A.class, name);
			limelight.setPollRateHz(pollHz);
			limelight.start();
			imu = hardwareMap.get(IMU.class, "imu");

			initialized = true;
			return true;
		} catch (Exception e)
		{
			limelight = null;
			initialized = false;
			return false;
		}
	}

	public static void update()
	{
		YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
		double yaw = orientation.getYaw(AngleUnit.DEGREES);

		limelight.updateRobotOrientation(yaw);
	}

	/**
	 * Stop the limelight
	 */
	public static void stop ()
	{

		if (limelight != null)
		{
			try
			{
				limelight.stop();
			} catch (Exception e)
			{
				// Ignore errors during shutdown
			}
		}
		initialized = false;
	}

	/**
	 * Check if limelight is initialized and ready
	 */
	public static boolean isInitialized ()
	{

		return initialized && limelight != null;
	}

	/**
	 * Switch Limelight pipeline
	 */
	public static void setPipeline (int pipeline)
	{

		if (isInitialized())
		{
			limelight.pipelineSwitch(pipeline);
		}
	}

	/**
	 * Set Limelight LED mode
	 * 0=pipeline, 1=off, 2=blink, 3=on
	 */
	public static void setLedMode (int mode)
	{

		if (isInitialized())
		{
			// Implementation depends on Limelight3A API
			// limelight.setLedMode(mode);
		}
	}

	/**
	 * Get latest LLResult (may be null)
	 */
	public static LLResult getResult ()
	{

		if (!isInitialized()) return null;
		try
		{
			return limelight.getLatestResult();
		} catch (Exception e)
		{
			return null;
		}
	}

	/**
	 * Check if Limelight has a valid target
	 *
	 * @return Limelight sees a valid target
	 */
	public static boolean hasValidTarget ()
	{

		LLResult result = getResult();
		return result != null && result.isValid();
	}

	/**
	 * Get horizontal offset (tx) in degrees, or 0 if unavailable
	 */
	public static double getTx ()
	{

		LLResult result = getResult();
		return (result != null && result.isValid()) ? result.getTx() : 0;
	}

	/**
	 * Get vertical offset (ty) in degrees, or 0 if unavailable
	 */
	public static double getTy ()
	{

		LLResult result = getResult();
		return (result != null && result.isValid()) ? result.getTy() : 0;
	}

	/**
	 * Get target area (ta) as a fraction (0-1), or 0 if unavailable
	 */
	public static double getTa ()
	{

		LLResult result = getResult();
		return (result != null && result.isValid()) ? result.getTa() : 0;
	}

	/**
	 * Get distance to target in inches, or 0 if unavailable
	 */
	public static double getTargetDistance ()
	{

		LLResult result = getResult();
		return (result != null && result.isValid()) ? result.getBotposeAvgDist() * 39.3701 : 0;
	}

	/**
	 * Get the underlying Limelight3A instance for advanced operations
	 */
	public static Limelight3A getLimelight ()
	{

		return limelight;
	}
}
