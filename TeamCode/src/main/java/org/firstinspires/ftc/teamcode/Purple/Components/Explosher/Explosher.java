package org.firstinspires.ftc.teamcode.Purple.Components.Explosher;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

public class Explosher
{
    public enum DistanceState
    {
        NEAR, MID, FAR, AUTO;

        /**
         * Returns the next distance state in the cycle (NEAR -> MID -> FAR -> NEAR).
         *
         * @return The next DistanceState.
         */
        public DistanceState next()
        {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    private final DcMotorEx motor;
    private final MotorConfig motorConfig;
    private final Servo finger;
    private final ServoConfig fingerConfig;
    private DistanceState distanceState = DistanceState.NEAR;

    // Distance positions for the finger servo
    private static final double NEAR_POS = 0.0;
    private static final double MID_POS = 0.2;
    private static final double FAR_POS = 0.3;
    public static final int CLOSE_SWEET = 240;
    public static final int FAR_SWEET = 390;


    /**
     * Constructs an Explosher subsystem with motor and finger servo.
     * @param motor The DcMotorEx for the Explosher.
     * @param motorConfig The configuration for the motor.
     * @param finger The Servo for the finger.
     * @param fingerConfig The configuration for the finger servo.
     */
    public Explosher(DcMotorEx motor, MotorConfig motorConfig, Servo finger, ServoConfig fingerConfig)
    {
        this.motor = motor;
        this.motorConfig = motorConfig;
        this.finger = finger;
        this.fingerConfig = fingerConfig;
        motor.setDirection(motorConfig.getDirection());
        motor.setZeroPowerBehavior(motorConfig.getZeroPowerBehavior());
        motor.setMode(motorConfig.getRunMode());
        setMotorState(MotorConfig.MotorState.OFF);
        setFingerState(ServoConfig.ServoState.OFF);
        setDistanceState(DistanceState.NEAR);

        // Initialize timing and position tracking for RPM calculation
        motorConfig.initializeRPMTracking(motor.getCurrentPosition());
    }

    /**
     * Sets the state of the Explosher motor (ON/OFF).
     * @param state The desired MotorState.
     */
    public void setMotorState(MotorConfig.MotorState state)
    {
        motorConfig.setState(state);
        if (state == MotorConfig.MotorState.ON)
        {
            // If target RPM is set, use it; otherwise use default
            if (motorConfig.getTargetRPM() > 0) {
                setRPM(motorConfig.getTargetRPM());
            } else {
                setRPM(FAR_SWEET); // Default RPM
            }
        } else
        {
            motorConfig.setRPM(motor, 0);
        }
    }

    /**
     * Sets the motor RPM using setVelocity.
     * @param rpm The desired RPM.
     */
    public void setRPM(double rpm)
    {
        motorConfig.setRPM(motor, rpm);
    }

    /**
     * Gets the current RPM of the motor.
     * @return The current RPM.
     */
    public double getCurrentRPM()
    {
        return motorConfig.getRPM();
    }

    /**
     * Gets the target RPM of the motor.
     * @return The target RPM.
     */
    public double getTargetRPM()
    {
        return motorConfig.getTargetRPM();
    }

    /**
     * Gets the current state of the Explosher motor.
     * @return The current MotorState.
     */
    public MotorConfig.MotorState getMotorState()
    {
        return motorConfig.getState();
    }

    /**
     * Sets the state of the finger servo (ON/OFF).
     * @param state The desired ServoState.
     */
    public void setFingerState(ServoConfig.ServoState state)
    {
        fingerConfig.setState(state);
        if (state == ServoConfig.ServoState.ON)
        {
            finger.setPosition(fingerConfig.getMaxPosition());
        } else
        {
            finger.setPosition(fingerConfig.getMinPosition());
        }
    }

    /**
     * Gets the current state of the finger servo.
     * @return The current ServoState.
     */
    public ServoConfig.ServoState getFingerState()
    {
        return fingerConfig.getState();
    }

    /**
     * Sets the finger servo to a specific position, clamped to allowed range.
     * @param position The desired position for the finger servo.
     */
    public void setFingerPosition(double position)
    {
        double clamped = fingerConfig.clamp(position);
        finger.setPosition(clamped);
    }

    /**
     * Gets the current position of the finger servo.
     * @return The current position of the finger servo.
     */
    public double getFingerPosition()
    {
        return finger.getPosition();
    }

    /**
     * Sets the Explosher distance state (NEAR, MID, FAR) and updates the finger position accordingly.
     * @param state The desired DistanceState.
     */
    public void setDistanceState(DistanceState state)
    {
        this.distanceState = state;
        switch (state)
        {
            case NEAR:
                setFingerPosition(NEAR_POS);
                break;
            case MID:
                setFingerPosition(MID_POS);
                break;
            case FAR:
                setFingerPosition(FAR_POS);
                break;
            case AUTO:
                AutoSwag();
                break;
        }
    }

    public void AutoSwag()
    {
        if (distanceState == DistanceState.AUTO) {
            double distance = LimeUtil.getTargetDistance();
            double clampedDist = MathUtil.clamp(distance, 0.0, 0.3);
            setFingerPosition(clampedDist);
            DebugUtil.logAdd("" + clampedDist);
            DebugUtil.logAdd("" + LimeUtil.getResult());
            DebugUtil.logAdd("" + LimeUtil.getTx());
        }
    }

    /**
     * Gets the current Explosher distance state.
     * @return The current DistanceState.
     */
    public DistanceState getDistanceState()
    {
        return distanceState;
    }

    /**
     * Update method that should be called regularly to calculate and log RPM.
     */
    public void Update()
    {
        // Calculate current RPM for monitoring
        motorConfig.updateRPM(motor.getCurrentPosition());

        // Update AUTO mode if active
        if (distanceState == DistanceState.AUTO) {
            AutoSwag();
        }

        // Debug information
        DebugUtil.logAdd("Explosher RPM: " + String.format("%.2f", getCurrentRPM()) +
                "/" + String.format("%.2f", getTargetRPM()));
    }

    /**
     * Cycles the Explosher distance state to the next value (NEAR -> MID -> FAR -> NEAR).
     */
    public void cycleDistanceState()
    {
        setDistanceState(distanceState.next());
    }
}
