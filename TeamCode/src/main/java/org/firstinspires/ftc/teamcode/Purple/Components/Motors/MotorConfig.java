package org.firstinspires.ftc.teamcode.Purple.Components.Motors;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simplified motor configuration class for basic power, velocity, and position control
 * Uses MotorEx for all motors to ensure consistent velocity measurement
 */
public final class MotorConfig
{
    private final String name;
    private final Position position;
    private final MotorEx motor;
    private final double maxRPM;
    private final double cpr;

    private ControlMode controlMode = ControlMode.RAW_POWER;
    private double targetRPM = 0.0;
    private double targetPower = 0.0;

    private MotorConfig(Builder builder)
    {
        name = builder.name;
        position = builder.position;
        maxRPM = builder.maxRPM;
        cpr = builder.cpr;

        if (builder.cpr != 0 && builder.maxRPM != 0)
        {
            motor = new MotorEx(builder.hardwareMap, builder.name, builder.cpr, builder.maxRPM);
        }
        else
        {
            motor = new MotorEx(builder.hardwareMap, builder.name);
        }

        motor.setInverted(builder.inverted);
        motor.setZeroPowerBehavior(builder.zeroPowerBehavior);

        if (builder.positionCoefficient > 0)
        {
            motor.setPositionCoefficient(builder.positionCoefficient);
        }

        if (builder.positionTolerance > 0)
        {
            motor.setPositionTolerance(builder.positionTolerance);
        }

        if (builder.distancePerPulse > 0 && builder.cpr > 0)
        {
            motor.setDistancePerPulse(builder.distancePerPulse);
        }

        setControlMode(builder.velocityEnabled ? ControlMode.VELOCITY_CONTROL : ControlMode.RAW_POWER);
    }

    /**
     * Converts RPM to ticks per second
     *
     * @param rpm Revolutions per minute
     * @return Ticks per second
     */
    private double rpmToTps(double rpm)
    {
        if (cpr == 0)
        {
            return 0.0;
        }

        return rpm * (cpr / 60.0);
    }

    /**
     * Converts ticks per second to RPM
     *
     * @param tps Ticks per second
     * @return Revolutions per minute
     */
    private double tpsToRpm(double tps)
    {
        if (cpr == 0)
        {
            return 0.0;
        }

        return tps * (60.0 / cpr);
    }

    /**
     * Gets the current target RPM
     *
     * @return Target RPM value
     */
    public double getTargetRPM ()
    {

        return targetRPM;
    }

    /**
     * Sets target RPM using velocity control or falls back to power control
     *
     * @param rpm The target RPM to set
     */
    public void setTargetRPM (double rpm)
    {

        this.targetRPM = rpm;

        if (controlMode == ControlMode.VELOCITY_CONTROL && maxRPM > 0)
        {
            // Convert RPM to ticks per second and set velocity
            double tps = rpmToTps(rpm);
            motor.setVelocity(tps);
        } else
        {
            // Fallback to power control
            double power = maxRPM > 0 ? rpm / maxRPM : Math.min(1.0, rpm / 1000.0);
            power = Math.max(-1.0, Math.min(1.0, power));
            setPower(power);
        }
    }

    /**
     * Gets the current motor power
     *
     * @return Current power value between -1.0 and 1.0
     */
    public double getPower ()
    {

        return motor.get();
    }

    /**
     * Sets raw power to the motor (-1.0 to 1.0)
     *
     * @param power Power value between -1.0 and 1.0
     */
    public void setPower (double power)
    {

        targetPower = power;
        motor.set(power);
        if (controlMode != ControlMode.POSITION_CONTROL)
        {
            this.targetRPM = power * maxRPM;
        }
    }

    /**
     * Gets the current RPM from motor velocity
     *
     * @return Current RPM value
     */
    public double getCurrentRPM ()
    {
        // Use MotorEx's getVelocity() which returns ticks per second
        double tps = motor.getVelocity();
        return tpsToRpm(tps);
    }

    /**
     * Gets the current control mode
     *
     * @return Current control mode
     */
    public ControlMode getControlMode ()
    {

        return controlMode;
    }

    /**
     * Sets the control mode for the motor
     *
     * @param mode Control mode to set
     */
    public void setControlMode (ControlMode mode)
    {

        this.controlMode = mode;
        switch (mode)
        {
            case VELOCITY_CONTROL:
                motor.setRunMode(Motor.RunMode.VelocityControl);
                break;
            case POSITION_CONTROL:
                motor.setRunMode(Motor.RunMode.PositionControl);
                break;
            case RAW_POWER:
            default:
                motor.setRunMode(Motor.RunMode.RawPower);
                break;
        }
    }

    // ==================== POSITION CONTROL METHODS ====================

    /**
     * Gets the current position coefficient
     *
     * @return Current position coefficient value
     */
    public double getPositionCoefficient ()
    {

        return motor.getPositionCoefficient();
    }

    /**
     * Sets the position coefficient (kP) for position control
     *
     * @param coefficient Position coefficient value
     */
    public void setPositionCoefficient (double coefficient)
    {

        motor.setPositionCoefficient(coefficient);
    }

    /**
     * Sets the target position in encoder ticks
     *
     * @param target Desired position in ticks
     */
    public void setTargetPosition (int target)
    {

        motor.setTargetPosition(target);
    }

    /**
     * Sets the position tolerance in encoder ticks
     *
     * @param tolerance Allowed error in ticks
     */
    public void setPositionTolerance (double tolerance)
    {

        motor.setPositionTolerance(tolerance);
    }

    /**
     * Checks if the motor is at its target position
     *
     * @return true if within tolerance of target position
     */
    public boolean atTargetPosition ()
    {

        return motor.atTargetPosition();
    }

    /**
     * Gets the current encoder position
     *
     * @return Current position in ticks
     */
    public int getCurrentPosition ()
    {

        return motor.getCurrentPosition();
    }

    /**
     * Resets the encoder position to zero
     */
    public void resetEncoder ()
    {

        motor.resetEncoder();
    }

    /**
     * Sets the distance traveled per encoder pulse (in units of your choice)
     * This enables distance-based position control
     *
     * @param distancePerPulse Distance traveled per encoder tick
     */
    public void setDistancePerPulse (double distancePerPulse)
    {

        motor.setDistancePerPulse(distancePerPulse);
    }

    /**
     * Sets a target distance to travel
     * Requires distancePerPulse to be configured first
     *
     * @param distance Target distance to travel
     */
    public void setTargetDistance (double distance)
    {

        motor.setTargetDistance(distance);
    }

    /**
     * Gets the current distance traveled
     *
     * @return Current distance traveled
     */
    public double getDistance ()
    {

        return motor.getDistance();
    }

    /**
     * Runs the motor to a specific position with given power
     * This method must be called in a control loop when using position control
     *
     * @param targetPosition Target position in encoder ticks
     * @param power          Power to apply (0.0 to 1.0)
     */
    public void runToPosition (int targetPosition, double power)
    {

        if (getControlMode() != ControlMode.POSITION_CONTROL)
        {
            setControlMode(ControlMode.POSITION_CONTROL);
        }

        setTargetPosition(targetPosition);
        setPower(power);
    }

    /**
     * Runs the motor to a specific distance with given power
     * This method must be called in a control loop when using position control
     *
     * @param targetDistance Target distance to travel
     * @param power          Power to apply (0.0 to 1.0)
     */
    public void runToDistance (double targetDistance, double power)
    {

        if (getControlMode() != ControlMode.POSITION_CONTROL)
        {
            setControlMode(ControlMode.POSITION_CONTROL);
        }

        setTargetDistance(targetDistance);
        setPower(power);
    }

    /**
     * Gets the motor name
     *
     * @return Motor name
     */
    public String getName ()
    {

        return name;
    }

    /**
     * Gets the motor position
     *
     * @return Motor position
     */
    public Position getPosition ()
    {

        return position;
    }

    /**
     * Stops the motor
     */
    public void stop ()
    {

        motor.stopMotor();
        targetRPM = 0;
        targetPower = 0;
    }

    /**
     * Gets the maximum RPM capability
     *
     * @return Maximum RPM value
     */
    public double getMaxRPM ()
    {

        return maxRPM;
    }

    /**
     * Gets the counts per revolution (CPR)
     *
     * @return CPR value
     */
    public double getCPR ()
    {

        return cpr;
    }

    /**
     * Updates motor state - call in main loop for velocity and position control
     */
    public void update ()
    {
        // For position control, we need to keep calling set() with power
        if (controlMode == ControlMode.POSITION_CONTROL)
        {
            if (!atTargetPosition())
            {
                motor.set(targetPower);
            } else
            {
                // Stop when at target
                motor.set(0.0);
            }
        }
        // MotorEx handles velocity control updates internally
    }

    // ==================== ENUMS ====================

    public enum Position
    {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, EXPLOSHER, INTAKE, MIDTAKE
    }

    public enum ControlMode
    {
        RAW_POWER, VELOCITY_CONTROL, POSITION_CONTROL
    }

    // ==================== BUILDER ====================

    /**
     * Builder class for MotorConfig
     */
    public static class Builder
    {
        private final HardwareMap hardwareMap;
        private final String name;
        private final Position position;
        private final double maxRPM;
        private final double cpr;

        private boolean inverted = false;
        private Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
        private boolean velocityEnabled = true;
        private double positionCoefficient = 0.05; // Default kP for position control
        private double positionTolerance = 13.6;   // Default tolerance in ticks
        private double distancePerPulse = 0;       // Default: no distance per pulse configured

        /**
         * Creates a new MotorConfig builder with CPR and max RPM
         *
         * @param hw       Hardware map
         * @param name     Motor name
         * @param position Motor position
         * @param cpr      Counts per revolution
         * @param maxRPM   Maximum RPM for velocity control
         */
        public Builder (HardwareMap hw, String name, Position position, double cpr, double maxRPM)
        {

            this.hardwareMap = hw;
            this.name = name;
            this.position = position;
            this.cpr = cpr;
            this.maxRPM = maxRPM;
        }

        /**
         * Creates a new MotorConfig builder without CPR and max RPM (power control only)
         *
         * @param hw       Hardware map
         * @param name     Motor name
         * @param position Motor position
         */
        public Builder (HardwareMap hw, String name, Position position)
        {

            this(hw, name, position, 0, 0);
        }

        /**
         * Sets motor direction as inverted
         *
         * @return Builder instance
         */
        public Builder inverted ()
        {

            this.inverted = true;
            return this;
        }

        /**
         * Sets zero power behavior
         *
         * @param zeroPowerBehavior Zero power behavior to set
         * @return Builder instance
         */
        public Builder zeroPowerBehavior (Motor.ZeroPowerBehavior zeroPowerBehavior)
        {

            this.zeroPowerBehavior = zeroPowerBehavior;
            return this;
        }

        /**
         * Disables velocity control (uses raw power instead)
         *
         * @return Builder instance
         */
        public Builder disableVelocityControl ()
        {

            this.velocityEnabled = false;
            return this;
        }

        /**
         * Sets the position coefficient for position control
         *
         * @param coefficient Position coefficient (kP)
         * @return Builder instance
         */
        public Builder setPositionCoefficient (double coefficient)
        {

            this.positionCoefficient = coefficient;
            return this;
        }

        /**
         * Sets the position tolerance for position control
         *
         * @param tolerance Position tolerance in encoder ticks
         * @return Builder instance
         */
        public Builder setPositionTolerance (double tolerance)
        {

            this.positionTolerance = tolerance;
            return this;
        }

        /**
         * Sets the distance per pulse for distance-based position control
         * Typically: (wheel circumference) / (encoder counts per revolution)
         *
         * @param distancePerPulse Distance traveled per encoder tick
         * @return Builder instance
         */
        public Builder setDistancePerPulse (double distancePerPulse)
        {

            this.distancePerPulse = distancePerPulse;
            return this;
        }

        /**
         * Builds the MotorConfig instance
         *
         * @return Configured MotorConfig instance
         */
        public MotorConfig build ()
        {

            return new MotorConfig(this);
        }
    }
}


