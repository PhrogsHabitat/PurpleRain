package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;

public final class Constants
{
	// Finger servo configurations
	public static final ServoConfig FINGER_SERVO_CONFIG = new ServoConfig(Names.FINGER, 0.0, 1.0, 0.0);
	public static final double FINGER_STOP_POSITION = 0.48;    // Finger closed/stops pixel
	public static final double FINGER_PASS_POSITION = 0.69;    // Finger open/passes pixel
	public static final double FINGER_DEBUG_INCREMENT = 0.01; // Manual adjustment step

	// Drive settings
	public static final double DRIVE_POWER_SCALE = 1.0;
	public static final double DRIVE_POWER_BOOST = 2.0;
	public static final double JOYSTICK_DEADZONE = 0.1;
	public static final double LL_HEIGHT = 13.5;
	public static final double LL_ANGLE = 85;
	public static final double TARGET_HEIGHT = 29.5;
	public static final double DESIRED_TAG_DISTANCE = 18.0;
	public static final double ALIGN_ANGLE_KP = 0.5;
	public static final double ALIGN_DISTANCE_KP = 0.5; // Needs tuning!!!!!!
	public static final double ALIGN_STRAFE_KP   = 0.5; // Needs tuning!!!!!!

	public static final double MAX_ALIGN_POWER = 0.6;
	public static final double ALIGN_ANGLE_TOLERANCE = 1.0;
	public static final double ALIGN_DISTANCE_TOLERANCE = 1.0;
	public static final double ALIGN_ANGLE_DEADZONE = 0.5;
	public static final double ALIGN_DISTANCE_DEADZONE = 0.5;   // Beans
	// I2C address
	public static final I2cAddr ODOMETRY_COMPUTER_I2C_ADDR = I2cAddr.create7bit(0x31);
	// Vibration stuffies
	public static final int VIBRATION_TAG_DETECTED = 40;
	public static final int VIBRATION_ALIGNED = 100;
	// Note: MotorConfig instances will be created in the opmodes with HardwareMap
	// Debug mode toggle
	public static final boolean DEBUG_MODE = true;

	// 63, 2000
	// 90, 2300
	// 122, 2500
	private Constants ()
	{

	}
}