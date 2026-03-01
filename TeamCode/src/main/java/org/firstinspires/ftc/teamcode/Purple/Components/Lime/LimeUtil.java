package org.firstinspires.ftc.teamcode.Purple.Components.Lime;

import java.util.List;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Purple.Names;

public final class LimeUtil
{
    private static final double METERS_TO_INCHES = 39.3701;

    private static Limelight3A limelight = null;
    private static IMU imu = null;
    private static boolean initialized = false;

    private LimeUtil()
    {
    }

    /**
     * Initializes Limelight and IMU orientation source.
     *
     * @param hardwareMap FTC hardware map.
     * @param pollHz      Limelight poll rate.
     * @return True when initialization succeeds.
     */
    public static boolean start(HardwareMap hardwareMap, int pollHz)
    {
        try
        {
            limelight = hardwareMap.get(Limelight3A.class, Names.LIME);
            imu = hardwareMap.get(IMU.class, "imu");
            limelight.setPollRateHz(pollHz);
            limelight.start();
            initialized = true;
            return true;
        }
        catch (Exception ignored)
        {
            limelight = null;
            imu = null;
            initialized = false;
            return false;
        }
    }

    /**
     * Updates robot orientation input for Limelight.
     */
    public static void update()
    {
        if (!isInitialized() || imu == null)
        {
            return;
        }

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(yaw);
    }

    /**
     * Stops Limelight and clears initialization state.
     */
    public static void stop()
    {
        if (limelight != null)
        {
            try
            {
                limelight.stop();
            }
            catch (Exception ignored)
            {
            }
        }

        limelight = null;
        imu = null;
        initialized = false;
    }

    /**
     * Gets whether Limelight is initialized and available.
     *
     * @return True when initialized.
     */
    public static boolean isInitialized()
    {
        return initialized && limelight != null;
    }

    /**
     * Switches active Limelight pipeline.
     *
     * @param pipeline Pipeline index.
     */
    public static void setPipeline(int pipeline)
    {
        if (isInitialized())
        {
            limelight.pipelineSwitch(pipeline);
        }
    }

    /**
     * Sets Limelight LED mode.
     *
     * @param mode LED mode value.
     */
    public static void setLedMode(int mode)
    {
        if (isInitialized())
        {
            // Limelight3A SDK wrapper currently does not expose LED mode in this project.
        }
    }

    /**
     * Gets latest Limelight result.
     *
     * @return Latest result or null when unavailable.
     */
    public static LLResult getResult()
    {
        if (!isInitialized())
        {
            return null;
        }

        try
        {
            return limelight.getLatestResult();
        }
        catch (Exception ignored)
        {
            return null;
        }
    }

    /**
     * Gets whether Limelight has a valid target.
     *
     * @return True when a valid target is present.
     */
    public static boolean hasValidTarget()
    {
        LLResult result = getResult();
        return result != null && result.isValid();
    }

    /**
     * Gets horizontal target offset in degrees.
     *
     * @return Horizontal offset or 0 when unavailable.
     */
    public static double getTx()
    {
        LLResult result = getResult();
        return (result != null && result.isValid()) ? result.getTx() : 0.0;
    }

    /**
     * Gets vertical target offset in degrees.
     *
     * @return Vertical offset or 0 when unavailable.
     */
    public static double getTy()
    {
        LLResult result = getResult();
        return (result != null && result.isValid()) ? result.getTy() : 0.0;
    }

    /**
     * Gets target area fraction.
     *
     * @return Target area or 0 when unavailable.
     */
    public static double getTa()
    {
        LLResult result = getResult();
        return (result != null && result.isValid()) ? result.getTa() : 0.0;
    }

    /**
     * Gets estimated target distance in inches.
     *
     * @return Distance in inches or 0 when unavailable.
     */
    public static double getTargetDistance()
    {
        LLResult result = getResult();
        return (result != null && result.isValid()) ? result.getBotposeAvgDist() * METERS_TO_INCHES : 0.0;
    }

    /**
     * Gets first visible fiducial ID.
     *
     * @return Fiducial ID or -1 when unavailable.
     */
    public static int getPrimaryFiducialId()
    {
        LLResult result = getResult();
        if (result == null || !result.isValid())
        {
            return -1;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty())
        {
            return -1;
        }

        return fiducials.get(0).getFiducialId();
    }

    /**
     * Gets underlying Limelight hardware object.
     *
     * @return Limelight instance or null.
     */
    public static Limelight3A getLimelight()
    {
        return limelight;
    }
}


