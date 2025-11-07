// TODO: Why did I make this an interface again? Couldve just made it a base class but ok wtf
package org.firstinspires.ftc.teamcode.Purple.Components.Drive;

public interface DriveTrain
{
    void drive(double forward, double strafe, double turn);

    void stop();

    void setPowerScale(double scale);

    double getPowerScale();
}
