package org.firstinspires.ftc.teamcode.Purple.Components.Motors;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class MotorConfig {
    public enum Position {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, EXPLOSHER, INTAKE, MIDTAKE
    }

    public enum MotorState {
        ON, OFF
    }

    public enum ControlMode {
        RAW_POWER, VELOCITY_CONTROL, POSITION_CONTROL
    }

    private final String name;
    private final Position position;
    private final Motor motor;
    private MotorState state = MotorState.OFF;
    private ControlMode currentControlMode = ControlMode.VELOCITY_CONTROL;

    // PID coefficients
    private double veloP = 0.05;
    private double veloI = 0.01;
    private double veloD = 0.31;
    private double positionP = 0.05;

    private MotorConfig(Builder builder) {
        this.name = builder.name;
        this.position = builder.position;

        if (builder.useMotorEx)
        {
            this.motor = new MotorEx(builder.hardwareMap, builder.name);
        }
        else if (builder.maxRPM != 0 && builder.cpr != 0)
        {
            this.motor = new Motor(builder.hardwareMap, builder.name, builder.cpr, builder.maxRPM);
        }
        else
        {
            this.motor = new Motor(builder.hardwareMap, builder.name);
        }

        this.motor.setInverted(builder.inverted);
        this.motor.setZeroPowerBehavior(builder.zeroPowerBehavior);

        initializePIDCoefficients();
    }

    public static class Builder {
        private final HardwareMap hardwareMap;
        private final String name;
        private final Position position;
        private final double maxRPM;
        private final double cpr;

        private boolean inverted = false;
        private Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
        private boolean useMotorEx = false;

        public Builder(HardwareMap hardwareMap, String name, Position position, double cpr, double maxRPM) {
            this.hardwareMap = hardwareMap;
            this.name = name;
            this.position = position;
            this.maxRPM = maxRPM;
            this.cpr = cpr;
        }

        public Builder(HardwareMap hardwareMap, String name, Position position) {
            this.hardwareMap = hardwareMap;
            this.name = name;
            this.position = position;
            this.maxRPM = 0;
            this.cpr = 0;
        }

        public Builder inverted() {
            this.inverted = true;
            return this;
        }

        public Builder zeroPowerBehavior(Motor.ZeroPowerBehavior zeroPowerBehavior) {
            this.zeroPowerBehavior = zeroPowerBehavior;
            return this;
        }

        public Builder useMotorEx() {
            this.useMotorEx = true;
            return this;
        }

        public MotorConfig build() {
            return new MotorConfig(this);
        }
    }

    private void initializePIDCoefficients() {
//        motor.setVeloCoefficients(veloP, veloI, veloD);
//        motor.setPositionCoefficient(positionP);
    }

    // ===== VELOCITY METHODS =====
    public double getVelocity() {
        return motor.getCorrectedVelocity();
    }

    public void setTargetRPM(double rpm) {
        setControlMode(ControlMode.VELOCITY_CONTROL);
        motor.set(rpm);
        state = (rpm != 0) ? MotorState.ON : MotorState.OFF;
    }

    // ===== CONTROL MODE MANAGEMENT =====
    public void setControlMode(ControlMode controlMode) {
        this.currentControlMode = controlMode;
        switch (controlMode) {
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

    // ===== BASIC MOTOR CONTROL =====
    public void setPower(double power) {
        setControlMode(ControlMode.RAW_POWER);
        motor.set(power);
        state = (power != 0) ? MotorState.ON : MotorState.OFF;
    }

    public void stop() {
        motor.stopMotor();
        state = MotorState.OFF;
    }

    // ===== GETTER METHODS =====
    public String getName() { return name; }
    public Position getPosition() { return position; }
    public MotorState getState() { return state; }
    public ControlMode getControlMode() { return currentControlMode; }

    public void setState(MotorState state) {
        this.state = state;
        if (state == MotorState.OFF) stop();
    }
}