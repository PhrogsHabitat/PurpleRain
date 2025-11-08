package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;

public final class Constants {
    private Constants() {}

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

    // Drive settings
    public static final double DRIVE_POWER_SCALE = 1.0;
    public static final double DRIVE_POWER_BOOST = 2.0;
    public static final double JOYSTICK_DEADZONE = 0.1;

    public static final double LL_HEIGHT = 14.3;
    public static final double LL_ANGLE = 6.0;
    public static final double TARGET_HEIGHT = 29.5;

    public static final double DESIRED_TAG_DISTANCE = 18.0; // inches - adjust based on your needs
    public static final double ALIGN_ANGLE_KP = 0.02; // Proportional gain for angle correction
    public static final double ALIGN_DISTANCE_KP = 0.03; // Proportional gain for distance correction
    public static final double MAX_ALIGN_POWER = 0.4; // Maximum power during auto-align
    public static final double ALIGN_ANGLE_TOLERANCE = 1.0; // degrees
    public static final double ALIGN_DISTANCE_TOLERANCE = 1.0; // inches
    public static final double ALIGN_ANGLE_DEADZONE = 0.5; // degrees - ignore small errors
    public static final double ALIGN_DISTANCE_DEADZONE = 0.5; // inches - ignore small errors

    // I2C address of the GoBuilda Odometry Computer
    public static final I2cAddr ODOMETRY_COMPUTER_I2C_ADDR = I2cAddr.create7bit(0x31);

    // Explosher RPM values
    public static final int EXPLOSHER_CLOSE_SWEET = 240;
    public static final int EXPLOSHER_FAR_SWEET = 390;

    // Vibration patterns
    public static final int VIBRATION_TAG_DETECTED = 40;
    public static final int VIBRATION_ALIGNED = 100;
    public static final int VIBRATION_IMPACT = 750;

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
    public static final ServoConfig FINGER_SERVO_CONFIG = new ServoConfig(FINGER_SERVO, 0.0, 1.0, 0.0);

    // Debug mode toggle
    public static final boolean DEBUG_MODE = true;
}