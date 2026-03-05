package org.firstinspires.ftc.teamcode.Purple.Components.Vaccum;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Ball;
import org.firstinspires.ftc.teamcode.Purple.Memory.Components.Motif;
import org.firstinspires.ftc.teamcode.Purple.Memory.PurpleMemory;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Vaccum
{
    public static final double DEFAULT_POW = 1.0;

    private static final int FINGER_COUNT = 3;
    private static final long FINGER_HOLD_MS = 500;
    private static final double FLICK_POSITION = 0.5;

    private final MotorConfig intakeMotor;
    private final ServoConfig[] fingers;
    private final FingerState[] fingerStates = new FingerState[]{
            FingerState.READY,
            FingerState.READY,
            FingerState.READY
    };
    private final long[] fingerStateStartTimesMs = new long[FINGER_COUNT];
    private final int[] shootFingerQueue = new int[FINGER_COUNT];

    private int shootQueueSize = 0;
    private int shootQueueIndex = 0;
    private int activeShootFinger = -1;
    private boolean shootInProgress = false;
    private boolean shootFullInProgress = false;
    private double currentPower = 0.0;

    public Vaccum(HardwareMap hardwareMap)
    {
        intakeMotor = new MotorConfig.Builder(hardwareMap, Names.INTAKE, MotorConfig.Position.INTAKE)
                .disableVelocityControl()
                .build();

        fingers = new ServoConfig[]{
                new ServoConfig.Builder(hardwareMap, Names.FINGER_1)
                        .setRange(Constants.FINGER_MIN, Constants.FINGER_MAX)
                        .reversed()
                        .build(),
                new ServoConfig.Builder(hardwareMap, Names.FINGER_2)
                        .setRange(Constants.FINGER_MIN, Constants.FINGER_MAX)
                        .reversed()
                        .build(),
                new ServoConfig.Builder(hardwareMap, Names.FINGER_3)
                        .setRange(Constants.FINGER_MIN, Constants.FINGER_MAX)
                        .reversed()
                        .build()
        };

        for (ServoConfig finger : fingers)
        {
            finger.setPosition(finger.getMinPosition());
        }

        stop();
    }

    /**
     * Updates intake motor state and finger timing state machines.
     */
    public void update()
    {
        intakeMotor.update();
        long nowMs = System.currentTimeMillis();

        for (int fingerIndex = 0; fingerIndex < FINGER_COUNT; fingerIndex++)
        {
            updateFingerState(fingerIndex, nowMs);
        }

        updateShootSequence();
        DebugUtil.logAdd("Finger positions: " + fingers[0].getPosition() + ", " + fingers[1].getPosition() + ", " + fingers[2].getPosition());
    }

    /**
     * Sets intake motor power.
     *
     * @param power Intake power from -1.0 to 1.0.
     */
    public void setPower(double power)
    {
        currentPower = power;
        intakeMotor.setPower(power);
    }

    /**
     * Gets current intake motor power.
     *
     * @return Current intake power.
     */
    public double getPower()
    {
        return currentPower;
    }

    /**
     * Stops the intake motor.
     */
    public void stop()
    {
        setPower(0.0);
    }

    /**
     * Gets current intake RPM.
     *
     * @return Current intake RPM.
     */
    public double getIntakeCurrentRPM()
    {
        return intakeMotor.getCurrentRPM();
    }

    /**
     * Gets a finger servo position by index.
     *
     * @param index Finger index (0-2).
     * @return Servo position, or 0 when index is invalid.
     */
    public double getFingerPosition(int index)
    {
        if (!isValidFingerIndex(index))
        {
            return 0.0;
        }

        return fingers[index].getPosition();
    }

    /**
     * Flicks one finger by index.
     *
     * @param index Finger index (0-2).
     */
    public void flickFinger(int index)
    {
        startFlick(index);
    }

    /**
     * Shoots one ball based on current motif and memory ordering.
     */
    public void shoot()
    {
        shootQueueSize = 0;
        shootQueueIndex = 0;
        activeShootFinger = -1;

        PurpleMemory memory = PurpleMemory.Instance;
        if (memory == null)
        {
            enqueueFinger(0);
            shootInProgress = shootQueueSize > 0;
            updateShootSequence();
            return;
        }

        Ball[] currentBalls = memory.curBalls();
        Ball[] desiredOrder = getDesiredOrder(memory.curMotif());
        boolean[] usedSlots = new boolean[FINGER_COUNT];

        for (Ball desiredBall : desiredOrder)
        {
            int slotIndex = findUnassignedSlot(currentBalls, usedSlots, desiredBall);
            if (slotIndex >= 0)
            {
                usedSlots[slotIndex] = true;
                enqueueFinger(slotIndex);
                break;
            }
        }

        if (shootQueueSize == 0)
        {
            for (int slot = 0; slot < FINGER_COUNT; slot++)
            {
                if (currentBalls[slot] != Ball.NONE)
                {
                    enqueueFinger(slot);
                    break;
                }
            }
        }

        shootInProgress = shootQueueSize > 0;
        updateShootSequence();
    }

    /**
     * Builds and runs a full motif-ordered shoot sequence.
     */
    public void shootFull()
    {
        shootQueueSize = 0;
        shootQueueIndex = 0;
        activeShootFinger = -1;
        shootFullInProgress = true;

        queueNextFullShootBall();
        shootInProgress = shootQueueSize > 0;
        updateShootSequence();
    }

    /**
     * Ejects all balls in a default pattern without relying on color sensor data.
     * This method is intended as a fallback when the color sensors or memory
     * system are not available. It simply queues every finger in physical order
     * (0, 1, 2) and runs the normal shoot sequence.
     */
    public void eject()
    {
        // reset queue state
        shootQueueSize = 0;
        shootQueueIndex = 0;
        activeShootFinger = -1;

        // default pattern is sequential finger order
        enqueueAllFingers();

        shootInProgress = shootQueueSize > 0;
        updateShootSequence();
    }

    /**
     * Queues the next ball to shoot in full shoot mode based on current memory state.
     * Re-checks the memory after each shot to handle cases where a ball wasn't expelled.
     */
    private void queueNextFullShootBall()
    {
        PurpleMemory memory = PurpleMemory.Instance;
        if (memory == null)
        {
            // If memory is null, fall back to unordered shooting
            for (int slot = 0; slot < FINGER_COUNT; slot++)
            {
                if (fingerStates[slot] == FingerState.READY)
                {
                    enqueueFinger(slot);
                    return;
                }
            }
            return;
        }

        Ball[] currentBalls = memory.curBalls();
        Ball[] desiredOrder = getDesiredOrder(memory.curMotif());
        boolean[] usedSlots = new boolean[FINGER_COUNT];

        // First pass: try to find a ball matching the desired order
        for (Ball desiredBall : desiredOrder)
        {
            int slotIndex = findUnassignedSlot(currentBalls, usedSlots, desiredBall);
            if (slotIndex >= 0)
            {
                usedSlots[slotIndex] = true;
                enqueueFinger(slotIndex);
                return;
            }
        }

        // Second pass: queue any remaining ball
        for (int slot = 0; slot < FINGER_COUNT; slot++)
        {
            if (currentBalls[slot] != Ball.NONE)
            {
                enqueueFinger(slot);
                return;
            }
        }

        // No more balls to shoot
        shootFullInProgress = false;
    }

    private boolean startFlick(int index)
    {
        if (!isValidFingerIndex(index) || fingerStates[index] != FingerState.READY)
        {
            return false;
        }

        fingers[index].setPosition(FLICK_POSITION);
        fingerStates[index] = FingerState.HOLDING_MAX;
        fingerStateStartTimesMs[index] = System.currentTimeMillis();
        return true;
    }

    private void updateFingerState(int index, long nowMs)
    {
        if (!isValidFingerIndex(index))
        {
            return;
        }

        if (fingerStates[index] == FingerState.HOLDING_MAX)
        {
            if (nowMs - fingerStateStartTimesMs[index] >= FINGER_HOLD_MS)
            {
                fingers[index].setPosition(fingers[index].getMinPosition());
                fingerStates[index] = FingerState.HOLDING_MIN;
                fingerStateStartTimesMs[index] = nowMs;
            }
            return;
        }

        if (fingerStates[index] == FingerState.HOLDING_MIN &&
                nowMs - fingerStateStartTimesMs[index] >= FINGER_HOLD_MS)
        {
            fingerStates[index] = FingerState.READY;
        }
    }

    private void updateShootSequence()
    {
        if (!shootInProgress)
        {
            return;
        }

        if (activeShootFinger != -1)
        {
            if (fingerStates[activeShootFinger] != FingerState.READY)
            {
                return;
            }

            // After a finger finishes, if we're in full shoot mode, queue the next ball
            if (shootFullInProgress)
            {
                queueNextFullShootBall();
            }

            activeShootFinger = -1;
        }

        if (shootQueueIndex >= shootQueueSize)
        {
            shootInProgress = false;
            shootFullInProgress = false;
            return;
        }

        int fingerIndex = shootFingerQueue[shootQueueIndex];
        if (startFlick(fingerIndex))
        {
            activeShootFinger = fingerIndex;
            shootQueueIndex++;
        }
    }

    private void enqueueAllFingers()
    {
        for (int finger = 0; finger < FINGER_COUNT; finger++)
        {
            enqueueFinger(finger);
        }
    }

    private void enqueueFinger(int fingerIndex)
    {
        if (!isValidFingerIndex(fingerIndex) || shootQueueSize >= shootFingerQueue.length)
        {
            return;
        }

        shootFingerQueue[shootQueueSize++] = fingerIndex;
    }

    private int findUnassignedSlot(Ball[] currentBalls, boolean[] usedSlots, Ball desiredBall)
    {
        for (int slot = 0; slot < FINGER_COUNT; slot++)
        {
            if (!usedSlots[slot] && currentBalls[slot] == desiredBall)
            {
                return slot;
            }
        }

        return -1;
    }

    private Ball[] getDesiredOrder(Motif.Type motif)
    {
        switch (motif)
        {
            case PGP:
                return new Ball[]{Ball.PURPLE, Ball.GREEN, Ball.PURPLE};
            case PPG:
                return new Ball[]{Ball.PURPLE, Ball.PURPLE, Ball.GREEN};
            case GPP:
            default:
                return new Ball[]{Ball.GREEN, Ball.PURPLE, Ball.PURPLE};
        }
    }

    private boolean isValidFingerIndex(int index)
    {
        return index >= 0 && index < FINGER_COUNT;
    }

    private enum FingerState
    {
        READY,
        HOLDING_MAX,
        HOLDING_MIN
    }
}

