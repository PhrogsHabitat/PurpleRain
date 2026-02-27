package org.firstinspires.ftc.teamcode.Purple.Components.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Configuration wrapper for a color sensor.
 */
public final class SensorConfig
{
    private static final double DEFAULT_PURPLE_THRESHOLD = 0.6;
    private static final double DEFAULT_GREEN_THRESHOLD = 0.6;

    private final String name;
    private final ColorSensor sensor;

    private double purpleThreshold = DEFAULT_PURPLE_THRESHOLD;
    private double greenThreshold = DEFAULT_GREEN_THRESHOLD;

    /**
     * Constructs a color-sensor configuration wrapper.
     *
     * @param hardwareMap FTC hardware map.
     * @param name        Sensor name from robot config.
     */
    public SensorConfig(HardwareMap hardwareMap, String name)
    {
        this.name = name;
        sensor = hardwareMap.get(ColorSensor.class, name);
        enableLED(true);
    }

    /**
     * Gets the configured sensor name.
     *
     * @return Sensor name.
     */
    public String getName()
    {
        return name;
    }

    /**
     * Gets raw red value.
     *
     * @return Red channel value.
     */
    public int getRed()
    {
        return sensor.red();
    }

    /**
     * Gets raw green value.
     *
     * @return Green channel value.
     */
    public int getGreen()
    {
        return sensor.green();
    }

    /**
     * Gets raw blue value.
     *
     * @return Blue channel value.
     */
    public int getBlue()
    {
        return sensor.blue();
    }

    /**
     * Enables or disables the sensor LED.
     *
     * @param enable True to enable LED.
     */
    public void enableLED(boolean enable)
    {
        sensor.enableLed(enable);
    }

    /**
     * Gets normalized RGB values in the range [0, 1].
     *
     * @return Normalized RGB array.
     */
    public double[] getNormalizedRGB()
    {
        int red = getRed();
        int green = getGreen();
        int blue = getBlue();
        int total = red + green + blue;
        if (total == 0)
        {
            return new double[]{0.0, 0.0, 0.0};
        }

        return new double[]{
                red / (double) total,
                green / (double) total,
                blue / (double) total
        };
    }

    /**
     * Gets purple detection threshold.
     *
     * @return Purple threshold.
     */
    public double getPurpleThreshold()
    {
        return purpleThreshold;
    }

    /**
     * Sets purple detection threshold.
     *
     * @param threshold Purple threshold.
     */
    public void setPurpleThreshold(double threshold)
    {
        purpleThreshold = threshold;
    }

    /**
     * Gets green detection threshold.
     *
     * @return Green threshold.
     */
    public double getGreenThreshold()
    {
        return greenThreshold;
    }

    /**
     * Sets green detection threshold.
     *
     * @param threshold Green threshold.
     */
    public void setGreenThreshold(double threshold)
    {
        greenThreshold = threshold;
    }
}


