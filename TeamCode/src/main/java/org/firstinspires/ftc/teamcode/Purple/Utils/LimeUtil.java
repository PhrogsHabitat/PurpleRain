package org.firstinspires.ftc.teamcode.Purple.Utils;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * FTC Limelight utility: wraps Limelight3A for easy, robust access
 * Consolidated version with static initialization and access methods
 */
public final class LimeUtil {
    // Configuration constants
    public static final double CAMERA_HEIGHT = 13.125; // Inches from floor
    public static final double TARGET_HEIGHT = 6.5; // Inches (target height)
    public static final double CAMERA_MOUNT_ANGLE = 45.0; // Degrees

    // Limelight instance
    private static Limelight3A limelight = null;
    private static boolean initialized = false;

    private LimeUtil() {
        // Utility class - prevent instantiation
    }

    /**
     * Initialize Limelight3A from hardwareMap
     *
     * @param hardwareMap FTC hardwareMap
     * @param name device name (e.g. "Limelight")
     * @param pollHz polling rate
     * @return true if initialization successful
     */
    public static boolean start(HardwareMap hardwareMap, String name, int pollHz) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, name);
            limelight.setPollRateHz(pollHz);
            limelight.start();
            initialized = true;
            return true;
        } catch (Exception e) {
            limelight = null;
            initialized = false;
            return false;
        }
    }

    /**
     * Initialize with default polling rate (60 Hz)
     */
    public static boolean start(HardwareMap hardwareMap, String name) {
        return start(hardwareMap, name, 60);
    }

    /**
     * Stop the limelight
     */
    public static void stop() {
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception e) {
                // Ignore errors during shutdown
            }
        }
        initialized = false;
    }

    /**
     * Check if limelight is initialized and ready
     */
    public static boolean isInitialized() {
        return initialized && limelight != null;
    }

    /**
     * Switch Limelight pipeline
     */
    public static void setPipeline(int pipeline) {
        if (isInitialized()) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    /**
     * Set Limelight LED mode
     * 0=pipeline, 1=off, 2=blink, 3=on
     */
    public static void setLedMode(int mode) {
        if (isInitialized()) {
            // Implementation depends on Limelight3A API
            // limelight.setLedMode(mode);
        }
    }

    /**
     * Get latest LLResult (may be null)
     */
    public static LLResult getResult() {
        if (!isInitialized()) return null;
        try {
            return limelight.getLatestResult();
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * True if Limelight sees a valid target
     */
    public static boolean hasValidTarget() {
        LLResult result = getResult();
        return result != null && result.isValid();
    }

    /**
     * Get horizontal offset (tx) in degrees, or 0 if unavailable
     */
    public static double getTx() {
        LLResult result = getResult();
        return (result != null && result.isValid()) ? result.getTx() : 0;
    }

    /**
     * Get vertical offset (ty) in degrees, or 0 if unavailable
     */
    public static double getTy() {
        LLResult result = getResult();
        return (result != null && result.isValid()) ? result.getTy() : 0;
    }

    /**
     * Get target area (ta) as a fraction (0-1), or 0 if unavailable
     */
    public static double getTa() {
        LLResult result = getResult();
        return (result != null && result.isValid()) ? result.getTa() : 0;
    }

    /**
     * Calculate distance to target using ty and trigonometry
     * Returns -1 if no valid target
     */
    public static double getTargetDistance() {
        if (!hasValidTarget()) return -1;
        double ty = getTy();
        double angleToTarget = Math.toRadians(ty + CAMERA_MOUNT_ANGLE);
        if (Math.abs(Math.tan(angleToTarget)) < 1e-6) return -1; // avoid div0
        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToTarget);
    }

    /**
     * Get a formatted string with Limelight telemetry
     */
    public static String getTelemetry() {
        if (!hasValidTarget()) {
            return "Target: NO";
        }
        return String.format("Target: YES X: %.1f Y: %.1f Area: %.1f%% Dist: %.1f\"",
                getTx(), getTy(), getTa() * 100, getTargetDistance());
    }

    /**
     * Get the underlying Limelight3A instance for advanced operations
     */
    public static Limelight3A getLimelight() {
        return limelight;
    }
}