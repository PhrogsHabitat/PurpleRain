package org.firstinspires.ftc.teamcode.Purple.Components.Servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class ServoConfig
{
    private final String name;
    private final Servo servo;
    private final double minPosition;
    private final double maxPosition;

    private ServoState state = ServoState.OFF;

    private ServoConfig(Builder builder)
    {
        if (builder.minPosition > builder.maxPosition)
        {
            throw new IllegalArgumentException("Servo minPosition cannot be greater than maxPosition");
        }

        name = builder.name;
        minPosition = builder.minPosition;
        maxPosition = builder.maxPosition;
        servo = builder.hardwareMap.get(Servo.class, builder.name);
        servo.setDirection(builder.direction);
    }

    /**
     * Gets servo name.
     *
     * @return Servo name.
     */
    public String getName()
    {
        return name;
    }

    /**
     * Gets configured minimum position.
     *
     * @return Minimum servo position.
     */
    public double getMinPosition()
    {
        return minPosition;
    }

    /**
     * Gets configured maximum position.
     *
     * @return Maximum servo position.
     */
    public double getMaxPosition()
    {
        return maxPosition;
    }

    /**
     * Gets wrapped servo hardware object.
     *
     * @return Servo instance.
     */
    public Servo getServo()
    {
        return servo;
    }

    /**
     * Gets current servo position.
     *
     * @return Current servo position.
     */
    public double getPosition()
    {
        return servo.getPosition();
    }

    /**
     * Sets servo position, clamped to configured range.
     *
     * @param position Requested servo position.
     */
    public void setPosition(double position)
    {
        servo.setPosition(clamp(position));
    }

    /**
     * Gets logical servo state.
     *
     * @return Current servo state.
     */
    public ServoState getState()
    {
        return state;
    }

    /**
     * Sets logical servo state.
     *
     * @param state Desired servo state.
     */
    public void setState(ServoState state)
    {
        this.state = state;
    }

    /**
     * Clamps a position to configured servo range.
     *
     * @param position Position to clamp.
     * @return Clamped position.
     */
    public double clamp(double position)
    {
        return Math.max(minPosition, Math.min(maxPosition, position));
    }

    public enum ServoState
    {
        ON,
        OFF
    }

    public static class Builder
    {
        private final HardwareMap hardwareMap;
        private final String name;

        private double minPosition = 0.0;
        private double maxPosition = 1.0;
        private Servo.Direction direction = Servo.Direction.FORWARD;

        /**
         * Creates a servo builder.
         *
         * @param hardwareMap FTC hardware map.
         * @param name        Servo name from robot config.
         */
        public Builder(HardwareMap hardwareMap, String name)
        {
            this.hardwareMap = hardwareMap;
            this.name = name;
        }

        /**
         * Sets minimum servo position.
         *
         * @param minPosition Minimum position.
         * @return Builder instance.
         */
        public Builder setMinPosition(double minPosition)
        {
            this.minPosition = minPosition;
            return this;
        }

        /**
         * Sets maximum servo position.
         *
         * @param maxPosition Maximum position.
         * @return Builder instance.
         */
        public Builder setMaxPosition(double maxPosition)
        {
            this.maxPosition = maxPosition;
            return this;
        }

        /**
         * Sets servo position range.
         *
         * @param minPosition Minimum position.
         * @param maxPosition Maximum position.
         * @return Builder instance.
         */
        public Builder setRange(double minPosition, double maxPosition)
        {
            this.minPosition = minPosition;
            this.maxPosition = maxPosition;
            return this;
        }

        /**
         * Sets servo direction to reverse.
         *
         * @return Builder instance.
         */
        public Builder reversed()
        {
            direction = Servo.Direction.REVERSE;
            return this;
        }

        /**
         * Sets servo direction.
         *
         * @param direction Servo direction.
         * @return Builder instance.
         */
        public Builder direction(Servo.Direction direction)
        {
            this.direction = direction;
            return this;
        }

        /**
         * Builds the configured ServoConfig.
         *
         * @return ServoConfig instance.
         */
        public ServoConfig build()
        {
            return new ServoConfig(this);
        }
    }
}


