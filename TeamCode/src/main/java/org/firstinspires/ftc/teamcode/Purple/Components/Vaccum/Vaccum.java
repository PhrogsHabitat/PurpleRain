package org.firstinspires.ftc.teamcode.Purple.Components.Vaccum;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Vaccum {
    private final DcMotorEx intakeMotor;
    private final DcMotorEx midtakeMotor;
    private final MotorConfig intakeMotorConfig;
    private final MotorConfig midtakeMotorConfig;

    public static final double DEFAULT_RPM = 1500;

    public Vaccum(DcMotorEx intakeMotor, MotorConfig intakeMotorConfig, DcMotorEx midtakeMotor, MotorConfig midtakeMotorConfig) {
        this.intakeMotor = intakeMotor;
        this.intakeMotorConfig = intakeMotorConfig;
        this.midtakeMotor = midtakeMotor;
        this.midtakeMotorConfig = midtakeMotorConfig;

        this.intakeMotor.setDirection(intakeMotorConfig.getDirection());
        this.intakeMotor.setZeroPowerBehavior(intakeMotorConfig.getZeroPowerBehavior());
        this.intakeMotor.setMode(intakeMotorConfig.getRunMode());
        intakeMotorConfig.initializeRPMTracking(intakeMotor.getCurrentPosition());

        this.midtakeMotor.setDirection(midtakeMotorConfig.getDirection());
        this.midtakeMotor.setZeroPowerBehavior(midtakeMotorConfig.getZeroPowerBehavior());
        this.midtakeMotor.setMode(midtakeMotorConfig.getRunMode());
        midtakeMotorConfig.initializeRPMTracking(midtakeMotor.getCurrentPosition());


        setState(MotorConfig.MotorState.OFF);
    }

    public void setState(MotorConfig.MotorState state) {
        intakeMotorConfig.setState(state);
        midtakeMotorConfig.setState(state);
        if (state == MotorConfig.MotorState.ON) {
            setRPM(DEFAULT_RPM);
        } else {
            setRPM(0);
        }
    }

    public MotorConfig.MotorState getState() {
        // Assuming both motors will be in the same state
        return intakeMotorConfig.getState();
    }

    public void setRPM(double rpm) {
        intakeMotorConfig.setRPM(intakeMotor, rpm);
        midtakeMotorConfig.setRPM(midtakeMotor, rpm);
    }

    public double getIntakeCurrentRPM() {
        return intakeMotorConfig.getRPM();
    }

    public double getMidtakeCurrentRPM() {
        return midtakeMotorConfig.getRPM();
    }

    public double getTargetRPM() {
        // Assuming both motors will have the same target RPM
        return intakeMotorConfig.getTargetRPM();
    }

    public void update() {
        intakeMotorConfig.updateRPM(intakeMotor.getCurrentPosition());
        midtakeMotorConfig.updateRPM(midtakeMotor.getCurrentPosition());

        DebugUtil.logAdd("Intake RPM: " + String.format("%.2f", getIntakeCurrentRPM()) + "/" + String.format("%.2f", getTargetRPM()));
        DebugUtil.logAdd("Midtake RPM: " + String.format("%.2f", getMidtakeCurrentRPM()) + "/" + String.format("%.2f", getTargetRPM()));
    }
}
