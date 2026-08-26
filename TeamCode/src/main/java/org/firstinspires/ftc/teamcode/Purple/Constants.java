package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.hardware.I2cAddr;

public final class Constants
{

	// Blocker servo configuration values
	public static final double BLOCKER_MIN = 0.38;
	public static final double BLOCKER_MAX = 1.0;
	public static final double GATE_STOP_POSITION = 1.0;
	public static final double GATE_PASS_POSITION = 0.69;

	// Drive settings
	public static final double DRIVE_POWER_SCALE = 1.0;
	public static final double DRIVE_POWER_BOOST = 2.0;
	public static final double JOYSTICK_DEADZONE = 0.1;
	public static final double DESIRED_TAG_DISTANCE = 18.0;
	public static final double ALIGN_ANGLE_TOLERANCE = 1.0;
	public static final double ALIGN_DISTANCE_TOLERANCE = 1.0;
	public static final double AUTO_ROTATE_KP = 0.018;
	public static final double AUTO_ROTATE_MAX_TURN = 0.45;

	// I2C address
	public static final I2cAddr ODOMETRY_COMPUTER_I2C_ADDR = I2cAddr.create7bit(0x31);
	// Vibration stuffies
	public static final int VIBRATION_TAG_DETECTED = 40;
	public static final int VIBRATION_ALIGNED = 100;

	// Debug mode toggle
	public static final boolean DEBUG_MODE = false;

}
