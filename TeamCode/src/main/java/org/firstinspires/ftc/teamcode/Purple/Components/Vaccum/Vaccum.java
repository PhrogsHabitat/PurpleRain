package org.firstinspires.ftc.teamcode.Purple.Components.Vaccum;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Vaccum {
    private final MotorConfig intakeMotor;
    private final MotorConfig midtakeMotor;
    private double targetRPM;

    public static final double DEFAULT_POW = 1.0;

    public Vaccum(HardwareMap hardwareMap) {
        intakeMotor = new MotorConfig.Builder(hardwareMap, Names.INTAKE, MotorConfig.Position.INTAKE)
                .useMotorEx()
                .build();
        midtakeMotor = new MotorConfig.Builder(hardwareMap, Names.MIDTAKE, MotorConfig.Position.MIDTAKE)
                .useMotorEx()
                .build();

        setState(MotorConfig.MotorState.OFF);
    }

    public void setState(MotorConfig.MotorState state) {
        if (state == MotorConfig.MotorState.ON) {
            setPower(DEFAULT_POW);
        } else {
            setPower(0);
        }
    }

    public MotorConfig.MotorState getState() {
        return intakeMotor.getState();
    }

    public void setPower(double rpm) {
        this.targetRPM = rpm;
        intakeMotor.setPower(rpm / 2);
        midtakeMotor.setTargetRPM(rpm);
    }

    public double getIntakeCurrentRPM() {
        return intakeMotor.getVelocity();
    }

    public double getMidtakeCurrentRPM() {
        return midtakeMotor.getVelocity();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public void update() {
        // The update logic is now handled within the MotorConfig class
        DebugUtil.logAdd("Intake RPM: " + String.format("%.2f", getIntakeCurrentRPM()) + "/" + String.format("%.2f", getTargetRPM()));
        DebugUtil.logAdd("Midtake RPM: " + String.format("%.2f", getMidtakeCurrentRPM()) + "/" + String.format("%.2f", getTargetRPM()));
    }
}