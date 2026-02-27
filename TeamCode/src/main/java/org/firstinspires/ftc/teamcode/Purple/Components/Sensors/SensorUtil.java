package org.firstinspires.ftc.teamcode.Purple.Components.Sensors;

/**
 * Utility methods for color sensor processing.
 */
public final class SensorUtil
{
    public static final int BALL_NONE = 0;
    public static final int BALL_PURPLE = 1;
    public static final int BALL_GREEN = 2;

    private SensorUtil()
    {
    }

    /**
     * Detects whether normalized RGB values match purple.
     *
     * @param red       Normalized red value.
     * @param green     Normalized green value.
     * @param blue      Normalized blue value.
     * @param threshold Purple detection threshold.
     * @return True when detected as purple.
     */
    public static boolean isPurple(double red, double green, double blue, double threshold)
    {
        double purpleScore = (red + blue) / 2.0 - green;
        return purpleScore > threshold;
    }

    /**
     * Detects whether normalized RGB values match green.
     *
     * @param red       Normalized red value.
     * @param green     Normalized green value.
     * @param blue      Normalized blue value.
     * @param threshold Green detection threshold.
     * @return True when detected as green.
     */
    public static boolean isGreen(double red, double green, double blue, double threshold)
    {
        double greenScore = green - (red + blue) / 2.0;
        return greenScore > threshold;
    }

    /**
     * Detects ball color from normalized RGB values.
     *
     * @param red             Normalized red value.
     * @param green           Normalized green value.
     * @param blue            Normalized blue value.
     * @param purpleThreshold Purple detection threshold.
     * @param greenThreshold  Green detection threshold.
     * @return {@link #BALL_NONE}, {@link #BALL_PURPLE}, or {@link #BALL_GREEN}.
     */
    public static int detectBallColor(double red, double green, double blue, double purpleThreshold, double greenThreshold)
    {
        if (isPurple(red, green, blue, purpleThreshold))
        {
            return BALL_PURPLE;
        }

        if (isGreen(red, green, blue, greenThreshold))
        {
            return BALL_GREEN;
        }

        return BALL_NONE;
    }

    /**
     * Calculates normalized brightness from raw RGB values.
     *
     * @param red   Raw red value (0-255).
     * @param green Raw green value (0-255).
     * @param blue  Raw blue value (0-255).
     * @return Brightness value in range [0, 1].
     */
    public static double getBrightness(int red, int green, int blue)
    {
        return (red + green + blue) / (3.0 * 255.0);
    }

    /**
     * Smooths a value with exponential moving average.
     *
     * @param current  Current sample value.
     * @param previous Previous smoothed value.
     * @param alpha    Smoothing factor.
     * @return New smoothed value.
     */
    public static double smoothValue(double current, double previous, double alpha)
    {
        return alpha * current + (1 - alpha) * previous;
    }
}


