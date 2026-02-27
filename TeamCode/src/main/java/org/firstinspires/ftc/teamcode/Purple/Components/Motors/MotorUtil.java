package org.firstinspires.ftc.teamcode.Purple.Components.Motors;

public final class MotorUtil
{
    private MotorUtil()
    {
    }

    /**
     * Normalizes an array of motor powers so that no value exceeds 1.0 in magnitude.
     *
     * @param powers Array of motor powers.
     * @return Normalized array.
     */
    public static double[] normalizePowers(double[] powers)
    {
        double max = 0.0;
        for (double power : powers)
        {
            max = Math.max(max, Math.abs(power));
        }

        if (max > 1.0)
        {
            for (int i = 0; i < powers.length; i++)
            {
                powers[i] /= max;
            }
        }

        return powers;
    }

    /**
     * Clamps a value between min and max.
     *
     * @param value Value to clamp.
     * @param min   Minimum value.
     * @param max   Maximum value.
     * @return Clamped value.
     */
    public static double clamp(double value, double min, double max)
    {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Calculates motor RPM from encoder deltas and elapsed time.
     *
     * @param currentPosition    Current encoder position.
     * @param lastPosition       Previous encoder position.
     * @param currentTime        Current timestamp in milliseconds.
     * @param lastTime           Previous timestamp in milliseconds.
     * @param ticksPerRevolution Encoder ticks per motor revolution.
     * @return Calculated RPM.
     */
    public static double calculateRPM(int currentPosition, int lastPosition, long currentTime, long lastTime, double ticksPerRevolution)
    {
        if (lastTime > 0 && currentTime > lastTime)
        {
            long deltaTime = currentTime - lastTime;
            int deltaPosition = currentPosition - lastPosition;
            return (deltaPosition / (double) deltaTime) * 1000.0 * 60.0 / ticksPerRevolution;
        }

        return 0.0;
    }
}


