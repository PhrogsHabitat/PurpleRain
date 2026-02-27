package org.firstinspires.ftc.teamcode.Purple.Components.Drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;

@Deprecated
public class MechanumDrive implements DriveTrain
{
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private double powerScale = 1.0;

    public MechanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight)
    {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    @Override
    public void drive(double forward, double strafe, double turn)
    {
        double y = -forward;
        double x = strafe * 1.1;
        double rx = turn;
        double[] powers = MotorUtil.normalizePowers(new double[]{
                y - x + rx,
                y + x + rx,
                y + x - rx,
                y - x - rx
        });

        frontLeft.setPower(powers[0] * powerScale);
        backLeft.setPower(powers[1] * powerScale);
        frontRight.setPower(powers[2] * powerScale);
        backRight.setPower(powers[3] * powerScale);
    }

    @Override
    public void stop()
    {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    @Override
    public double getPowerScale()
    {
        return powerScale;
    }

    @Override
    public void setPowerScale(double scale)
    {
        powerScale = scale;
    }
}


