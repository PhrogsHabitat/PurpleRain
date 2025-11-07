package org.firstinspires.ftc.teamcode.Purple.Components.Drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorUtil;

public class MechanumDrive implements DriveTrain
{
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private double powerScale = 1.0;

    public MechanumDrive(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br)
    {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;

        // Motor directions are set in MotorConfig, so we don't set them here
    }

    @Override
    public void drive(double forward, double strafe, double turn)
    {
        // Apply the working mecanum algorithm
        double y = -forward; // Remember, Y stick value is reversed
        double x = strafe * 1.1; // Counteract imperfect strafing
        double rx = turn;

        // Calculate raw powers
        double[] powers = new double[]{
                (y - x + rx),
                (y + x + rx),
                (y + x - rx),
                (y - x - rx)
        };
        // Normalize powers

        powers = MotorUtil.normalizePowers(powers);

        // Apply power scaling
        powers[0] *= powerScale;
        powers[1] *= powerScale;
        powers[2] *= powerScale;
        powers[3] *= powerScale;

        // Set motor powers
        frontLeft.setPower(powers[0]);
        backLeft.setPower(powers[1]);
        frontRight.setPower(powers[2]);
        backRight.setPower(powers[3]);
    }

    @Override
    public void stop()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public void setPowerScale(double scale)
    {
        this.powerScale = scale;
    }

    @Override
    public double getPowerScale()
    {
        return this.powerScale;
    }
}