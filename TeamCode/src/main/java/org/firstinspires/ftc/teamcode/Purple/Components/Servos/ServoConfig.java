package org.firstinspires.ftc.teamcode.Purple.Components.Servos;

/**
 * Configuration and state management for a servo.
 */
public final class ServoConfig
{
    /**
     * Represents the state of the servo (ON/OFF).
     */
    public enum ServoState
    {
        ON, OFF
    }

    private final String name;
    private final double minPosition;
    private final double maxPosition;
    private final double initialPosition;
    private ServoState state = ServoState.OFF;

    /**
     * Constructs a ServoConfig with the given parameters.
     *
     * @param name            The name of the servo.
     * @param minPosition     The minimum allowed position.
     * @param maxPosition     The maximum allowed position.
     * @param initialPosition The initial position (clamped).
     */
    public ServoConfig(String name, double minPosition, double maxPosition, double initialPosition)
    {
        this.name = name;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.initialPosition = clamp(initialPosition);
    }

    /**
     * Gets the name of the servo.
     * @return The servo name.
     */
    public String getName()
    {
        return name;
    }

    /**
     * Gets the minimum allowed position for the servo.
     * @return The minimum position.
     */
    public double getMinPosition()
    {
        return minPosition;
    }

    /**
     * Gets the maximum allowed position for the servo.
     * @return The maximum position.
     */
    public double getMaxPosition()
    {
        return maxPosition;
    }

    /**
     * Gets the initial position for the servo.
     * @return The initial position.
     */
    public double getInitialPosition()
    {
        return initialPosition;
    }

    /**
     * Gets the current state of the servo (ON/OFF).
     * @return The servo state.
     */
    public ServoState getState()
    {
        return state;
    }

    /**
     * Sets the state of the servo (ON/OFF).
     * @param state The desired servo state.
     */
    public void setState(ServoState state)
    {
        this.state = state;
    }

    /**
     * Clamps a position to the allowed range for this servo.
     * @param position The position to clamp.
     * @return The clamped position.
     */
    public double clamp(double position)
    {
        return Math.max(minPosition, Math.min(maxPosition, position));
    }

    /**
     * Checks if the given position is at the minimum allowed position.
     * @param position The position to check.
     * @return True if at minimum, false otherwise.
     */
    public boolean isAtMin(double position)
    {
        return Math.abs(position - minPosition) < 1e-4;
    }

    /**
     * Checks if the given position is at the maximum allowed position.
     * @param position The position to check.
     * @return True if at maximum, false otherwise.
     */
    public boolean isAtMax(double position)
    {
        return Math.abs(position - maxPosition) < 1e-4;
    }
}
