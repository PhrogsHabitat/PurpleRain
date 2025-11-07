package org.firstinspires.ftc.teamcode.Purple.Components.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public final class MotorConfig
{
    /**
     * Represents the position of the motor on the robot.
     */
    public enum Position
    {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, EXPLOSHER, INTAKE, MIDTAKE
    }

    /**
     * Represents the state of the motor (ON/OFF).
     */
    public enum MotorState
    {
        ON, OFF
    }

    private final String name;
    private final Position position;
    private final DcMotor.Direction direction;
    private final DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    private final DcMotor.RunMode runMode;
    private final double ticksPerRevolution;
    private MotorState state = MotorState.OFF;

    private double targetRPM = 0;
    private double currentRPM = 0;
    private int lastPosition = 0;
    private long lastTime = 0;

    /**
     * Constructs a MotorConfig using the builder.
     *
     * @param builder The builder containing configuration parameters.
     */
    private MotorConfig(Builder builder)
    {
        this.name = builder.name;
        this.position = builder.position;
        this.direction = builder.direction;
        this.zeroPowerBehavior = builder.zeroPowerBehavior;
        this.runMode = builder.runMode;
        this.ticksPerRevolution = builder.ticksPerRevolution;
    }

    /**
     * Builder class for MotorConfig.
     */
    public static class Builder
    {
        private String name;
        private Position position;
        private DcMotor.Direction direction = DcMotor.Direction.FORWARD;
        private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private double ticksPerRevolution = 537.6; // Default for many FTC motors

        /**
         * Creates a new builder for MotorConfig.
         * @param name The name of the motor.
         * @param position The position of the motor.
         */
        public Builder(String name, Position position)
        {
            this.name = name;
            this.position = position;
        }

        /**
         * Sets the direction for the motor.
         * @param direction The motor direction.
         * @return The builder instance.
         */
        public Builder direction(DcMotor.Direction direction)
        {
            this.direction = direction;
            return this;
        }

        /**
         * Sets the zero power behavior for the motor.
         * @param zeroPowerBehavior The zero power behavior.
         * @return The builder instance.
         */
        public Builder zeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
        {
            this.zeroPowerBehavior = zeroPowerBehavior;
            return this;
        }

        /**
         * Sets the run mode for the motor.
         * @param runMode The run mode.
         * @return The builder instance.
         */
        public Builder runMode(DcMotor.RunMode runMode)
        {
            this.runMode = runMode;
            return this;
        }

        /**
         * Sets the encoder ticks per revolution for the motor.
         * @param ticks The number of ticks per revolution.
         * @return The builder instance.
         */
        public Builder ticksPerRevolution(double ticks)
        {
            this.ticksPerRevolution = ticks;
            return this;
        }

        /**
         * Builds the MotorConfig instance.
         * @return The MotorConfig.
         */
        public MotorConfig build()
        {
            return new MotorConfig(this);
        }
    }

    // RPM RELATED METHODS

    /**
     * Initializes the RPM tracking variables.
     * @param currentPosition The starting encoder position of the motor.
     */
    public void initializeRPMTracking(int currentPosition)
    {
        this.lastPosition = currentPosition;
        this.lastTime = System.currentTimeMillis();
    }

    /**
     * Updates the current RPM calculation. Should be called in a loop.
     * @param currentPosition The current encoder position of the motor.
     */
    public void updateRPM(int currentPosition)
    {
        long currentTime = System.currentTimeMillis();
        if (lastTime > 0 && currentTime > lastTime)
        {
            long deltaTime = currentTime - lastTime;
            int deltaPosition = currentPosition - lastPosition;
            this.currentRPM = (deltaPosition / (double)deltaTime) * 1000.0 * 60.0 / ticksPerRevolution;
        }
        else
        {
            this.currentRPM = 0.0;
        }
        this.lastPosition = currentPosition;
        this.lastTime = currentTime;
    }


    /**
     * Sets the motor's target RPM and updates the motor's velocity.
     * @param motor The DcMotorEx instance to control.
     * @param rpm The desired target RPM.
     */
    public void setRPM(DcMotorEx motor, double rpm)
    {
        this.targetRPM = rpm;
        if (state == MotorState.ON && rpm > 0)
        {
            double targetVelocity = (rpm / 60.0) * ticksPerRevolution;
            motor.setVelocity(targetVelocity);
        } else {
            motor.setVelocity(0); // Also stop motor if rpm is 0 or state is OFF
        }
    }


    public double getRPM()
    {
        return currentRPM;
    }

    public double getTargetRPM()
    {
        return targetRPM;
    }


    public String getName()
    {
        return name;
    }

    public Position getPosition()
    {
        return position;
    }

    public DcMotor.Direction getDirection()
    {
        return direction;
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior()
    {
        return zeroPowerBehavior;
    }

    public DcMotor.RunMode getRunMode()
    {
        return runMode;
    }

    public MotorState getState()
    {
        return state;
    }

    public void setState(MotorState state)
    {
        this.state = state;
    }
}
