package org.firstinspires.ftc.teamcode.Purple.Components.Explosher;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Utils.MathUtil;

public class Explosher {
    public enum DistanceState {
        NEAR, MID, FAR, AUTO;

        public DistanceState next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    private final MotorConfig motor;
    private final Servo finger;
    private final ServoConfig fingerConfig;
    private DistanceState distanceState = DistanceState.NEAR;
    private double targetRPM;

    private static final double NEAR_POS = 0.0;
    private static final double MID_POS = 0.2;
    private static final double FAR_POS = 0.3;
    public static final int CLOSE_SWEET = 1000;
    public static final int FAR_SWEET = 2400;

    public Explosher(HardwareMap hardwareMap, ServoConfig fingerConfig) {
        this.motor = new MotorConfig.Builder(hardwareMap, Names.EXPLOSHER, MotorConfig.Position.EXPLOSHER)
                .useMotorEx()
                .build();

        this.finger = hardwareMap.get(Servo.class, fingerConfig.getName());
        this.fingerConfig = fingerConfig;
        this.targetRPM = 0;

        setMotorState(MotorConfig.MotorState.OFF);
        setFingerState(ServoConfig.ServoState.OFF);
        setDistanceState(DistanceState.NEAR);
    }

    public void setMotorState(MotorConfig.MotorState state) {
        if (state == MotorConfig.MotorState.ON) {
            double rpmToSet = (this.targetRPM > 0) ? this.targetRPM : FAR_SWEET;
            setRPM(rpmToSet);
        } else {
            motor.stop();
        }
    }

    public void setRPM(double rpm) {
        this.targetRPM = rpm;
        motor.setTargetRPM(rpm);
    }

    public double getCurrentRPM() {
        return motor.getVelocity();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public MotorConfig.MotorState getMotorState() {
        return motor.getState();
    }

    public void setFingerState(ServoConfig.ServoState state) {
        fingerConfig.setState(state);
        if (state == ServoConfig.ServoState.ON) {
            finger.setPosition(fingerConfig.getMaxPosition());
        } else {
            finger.setPosition(fingerConfig.getMinPosition());
        }
    }

    public ServoConfig.ServoState getFingerState() {
        return fingerConfig.getState();
    }

    public void setFingerPosition(double position) {
        double clamped = fingerConfig.clamp(position);
        finger.setPosition(clamped);
    }

    public double getFingerPosition() {
        return finger.getPosition();
    }


    public void setDistanceState(DistanceState state) {
        this.distanceState = state;
        switch (state) {
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

    public void AutoSwag() {
        if (distanceState == DistanceState.AUTO) {
            double distance = LimeUtil.getTargetDistance();
            double clampedDist = MathUtil.clamp(distance, 0.0, 0.3);
            setFingerPosition(clampedDist);
            DebugUtil.logAdd("" + clampedDist);
            DebugUtil.logAdd("" + LimeUtil.getResult());
            DebugUtil.logAdd("" + LimeUtil.getTx());
        }
    }

    public DistanceState getDistanceState() {
        return distanceState;
    }

    public void update() {
        if (distanceState == DistanceState.AUTO) {
            AutoSwag();
        }
        DebugUtil.logAdd("Explosher RPM: " + String.format("%.2f", getCurrentRPM()) +
                "/" + String.format("%.2f", getTargetRPM()));
    }

    public void cycleDistanceState() {
        setDistanceState(distanceState.next());
    }
}