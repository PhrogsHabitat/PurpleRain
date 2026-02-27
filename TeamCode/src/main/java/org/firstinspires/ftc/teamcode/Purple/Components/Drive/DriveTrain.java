package org.firstinspires.ftc.teamcode.Purple.Components.Drive;

@Deprecated
public interface DriveTrain
{
    void drive(double forward, double strafe, double turn);

    void stop();

    double getPowerScale();

    void setPowerScale(double scale);
}


