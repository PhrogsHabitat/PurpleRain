package org.firstinspires.ftc.teamcode.Purple.Utils;

public class MathUtil
{
    /**
     * Clamps a value between a minimum and maximum.
     *
     * @param value The value to clamp.
     * @param min   The minimum allowed value.
     * @param max   The maximum allowed value.
     * @return The clamped value.
     */
    public static double clamp(double value, double min, double max)
    {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Linearly interpolates between two values.
     * @param a The start value.
     * @param b The end value.
     * @param t The interpolation factor (0.0 to 1.0).
     * @return The interpolated value.
     */
    public static double lerp(double a, double b, double t)
    {
        return a + (b - a) * t;
    }

    /**
     * Normalizes an angle to the range [-PI, PI].
     * @param angle The angle in radians.
     * @return The normalized angle.
     */
    public static double normalizeAngle(double angle)
    {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
