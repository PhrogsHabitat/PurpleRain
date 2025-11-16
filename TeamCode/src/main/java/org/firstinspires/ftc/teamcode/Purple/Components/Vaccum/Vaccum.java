package org.firstinspires.ftc.teamcode.Purple.Components.Vaccum;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Vaccum {

    private final MotorConfig intakeMotor;
    private final MotorConfig midtakeMotor;

    public static final double DEFAULT_POW = 1.0;
    private double currentPower = 0;

    public Vaccum(HardwareMap hardwareMap) {
        intakeMotor = new MotorConfig.Builder(
                hardwareMap,
                Names.INTAKE,
                MotorConfig.Position.INTAKE
        ).useMotorEx().build();

        midtakeMotor = new MotorConfig.Builder(
                hardwareMap,
                Names.MIDTAKE,
                MotorConfig.Position.MIDTAKE
        ).useMotorEx().build();

        setState(MotorConfig.MotorState.OFF);
    }

    /** Sets ON/OFF state using raw power (not RPM). */
    public void setState(MotorConfig.MotorState state) {
        if (state == MotorConfig.MotorState.ON) {
            setPower(DEFAULT_POW);
        } else {
            setPower(0);
        }
    }

    public MotorConfig.MotorState getState() {
        return (currentPower > 0) ? MotorConfig.MotorState.ON : MotorConfig.MotorState.OFF;
    }

    /** Sets *power*, not RPM. Power range is -1 to 1. */
    public void setPower(double power) {
        currentPower = power;

        intakeMotor.setPower(power);
        midtakeMotor.setPower(power);
    }

    public double getPower() {
        return currentPower;
    }

    public double getIntakeCurrentRPM() {
        return intakeMotor.getCurrentRPM();
    }

    public double getMidtakeCurrentRPM() {
        return midtakeMotor.getCurrentRPM();
    }

    public void update() {
        DebugUtil.logAdd("Intake RPM: " +
                String.format("%.1f", getIntakeCurrentRPM()));

        DebugUtil.logAdd("Midtake RPM: " +
                String.format("%.1f", getMidtakeCurrentRPM()));
    }
}
