package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;

public final class Constants
{
    private Constants()
    {
    }

    // Motor names
    public static final String FRONT_LEFT_MOTOR = "FL";
    public static final String FRONT_RIGHT_MOTOR = "FR";
    public static final String BACK_LEFT_MOTOR = "BL";
    public static final String BACK_RIGHT_MOTOR = "BR";
    public static final String EXPLOSHER_MOTOR = "EXPLOSHER";

    public static final String INTAKE_MOTOR = "FE";
    public static final String MIDTAKE_MOTOR = "BE";


    // Servo names
    public static final String FINGER_SERVO = "FINGER";

    // Default drive power scaling
    public static final double DRIVE_POWER_SCALE = 1.0;

    // Deadzone for joysticks
    public static final double JOYSTICK_DEADZONE = 0.1;

    // I2C address of the GoBuilda Odometry Computer
    public static final I2cAddr ODOMETRY_COMPUTER_I2C_ADDR = I2cAddr.create7bit(0x31); // Example address

    // PID gains for path following
    public static final double PATH_KP = 0.05;
    public static final double PATH_KI = 0.0;
    public static final double PATH_KD = 0.02;
    public static final double PATH_MAX_POWER = 0.1;
    public static final double PATH_POSITION_TOLERANCE = 2.0; // inches
    public static final double PATH_HEADING_TOLERANCE = 0.1; // radians

    // Motor configurations
    public static final MotorConfig FL_CONFIG = new MotorConfig.Builder(FRONT_LEFT_MOTOR, MotorConfig.Position.FRONT_LEFT)
            .direction(com.qualcomm.robotcore.hardware.DcMotor.Direction.FORWARD)
            .runMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .build();

    public static final MotorConfig FR_CONFIG = new MotorConfig.Builder(FRONT_RIGHT_MOTOR, MotorConfig.Position.FRONT_RIGHT)
            .direction(com.qualcomm.robotcore.hardware.DcMotor.Direction.FORWARD)
            .runMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .build();

    public static final MotorConfig BL_CONFIG = new MotorConfig.Builder(BACK_LEFT_MOTOR, MotorConfig.Position.BACK_LEFT)
            .direction(com.qualcomm.robotcore.hardware.DcMotor.Direction.FORWARD)
            .runMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .build();

    public static final MotorConfig BR_CONFIG = new MotorConfig.Builder(BACK_RIGHT_MOTOR, MotorConfig.Position.BACK_RIGHT)
            .direction(com.qualcomm.robotcore.hardware.DcMotor.Direction.FORWARD)
            .runMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .build();

    public static final MotorConfig EXPLOSHER_CONFIG = new MotorConfig.Builder(EXPLOSHER_MOTOR, MotorConfig.Position.EXPLOSHER)
            .direction(com.qualcomm.robotcore.hardware.DcMotor.Direction.FORWARD)
            .runMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER)
            .build();

    public static final MotorConfig INTAKE_CONFIG = new MotorConfig.Builder(INTAKE_MOTOR, MotorConfig.Position.INTAKE)
            .direction(com.qualcomm.robotcore.hardware.DcMotor.Direction.FORWARD)
            .runMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER)
            .build();

    public static final MotorConfig MIDTAKE_CONFIG = new MotorConfig.Builder(MIDTAKE_MOTOR, MotorConfig.Position.MIDTAKE)
            .direction(com.qualcomm.robotcore.hardware.DcMotor.Direction.FORWARD)
            .runMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER)
            .build();


    // Servo configurations
    public static final ServoConfig FINGER_SERVO_CONFIG = new ServoConfig(FINGER_SERVO, 0.0, 1.0, 0.0); // min=0, max=1, initial=0

    // Debug mode toggle
    public static final boolean DEBUG_MODE = true;
}