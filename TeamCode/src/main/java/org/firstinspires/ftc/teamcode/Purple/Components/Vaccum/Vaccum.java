package org.firstinspires.ftc.teamcode.Purple.Components.Vaccum;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Purple.Components.Motors.MotorConfig;
import org.firstinspires.ftc.teamcode.Purple.Components.Servos.ServoConfig;
import org.firstinspires.ftc.teamcode.Purple.Constants;
import org.firstinspires.ftc.teamcode.Purple.Names;
import org.firstinspires.ftc.teamcode.Purple.Utils.DebugUtil;

public class Vaccum
{
	public static final double DEFAULT_POW = 1.0;
	private static final double FLICK_TIME_SECONDS = 0.5; // Adjust based on servo speed

	public final MotorConfig intakeMotor;

	// Individual finger servos – much clearer than an array
	private final Servo finger1;
	public final Servo finger2;
	private final Servo finger3;
	private final ServoConfig fingerConfig;

	// State machine for each finger (still needed for timing)
	private enum FingerState { IDLE, FLICKING }
	private FingerState state1 = FingerState.IDLE;
	private FingerState state2 = FingerState.IDLE;
	private FingerState state3 = FingerState.IDLE;

	private final ElapsedTime timer1 = new ElapsedTime();
	private final ElapsedTime timer2 = new ElapsedTime();
	private final ElapsedTime timer3 = new ElapsedTime();

	private double currentPower = 0;

	public Vaccum(HardwareMap hardwareMap)
	{
		intakeMotor = new MotorConfig.Builder(hardwareMap, Names.INTAKE, MotorConfig.Position.INTAKE)
				.disableVelocityControl()
				.build();

		this.fingerConfig = Constants.FINGER_SERVO_CONFIG;

		// Map each finger to its own hardware name
		finger1 = hardwareMap.get(Servo.class, Names.FINGER_1);
		finger2 = hardwareMap.get(Servo.class, Names.FINGER_2);
		finger3 = hardwareMap.get(Servo.class, Names.FINGER_3);

		// Set initial positions to rest (min)

		finger3.getController().resetDeviceConfigurationForOpMode();

		finger1.getController().pwmEnable();
		finger2.getController().pwmEnable();
		finger3.getController().pwmEnable();

		DebugUtil.logAdd("AHHHHHH" + finger1.getController().getPwmStatus());
		DebugUtil.logAdd("AHHHHHH2" + finger1.getController().getServoPosition(3));

		finger1.getController().setServoPosition(3, 1.0);

		finger1.setPosition(0);
		finger2.setPosition(0);
		finger3.setPosition(0);

		stop();
	}

	// ==================== INTAKE MOTOR ====================

	public double getPower() { return currentPower; }

	public void setPower(double power)
	{
		currentPower = power;
		intakeMotor.setPower(power);
	}

	public void stop() { setPower(0); }

	public double getIntakeCurrentRPM() { return intakeMotor.getCurrentRPM(); }

	// ==================== FINGER CONTROL ====================

	// Simple direct position control (for testing)
	public void setFinger1Position(double pos) { finger1.setPosition(pos); }
	public void setFinger2Position(double pos) { finger2.setPosition(pos); }
	public void setFinger3Position(double pos) { finger3.setPosition(pos); }

	// Flick methods – start the flick sequence
	public void flickFinger1()
	{
		finger1.setPosition(180);
		state1 = FingerState.FLICKING;
		timer1.reset();
	}

	public void flickFinger2()
	{
		finger2.setPosition(fingerConfig.getMaxPosition());
		state2 = FingerState.FLICKING;
		timer2.reset();
	}

	public void flickFinger3()
	{
		finger3.setPosition(fingerConfig.getMaxPosition());
		state3 = FingerState.FLICKING;
		timer3.reset();
	}

	// Optional: flick by index if you prefer
	public void flickFinger(int index)
	{
		switch (index) {
			case 0: flickFinger1(); break;
			case 1: flickFinger2(); break;
			case 2: flickFinger3(); break;
		}
	}

	// ==================== UPDATE LOOP ====================

	public void update()
	{
		intakeMotor.update();

		// Finger 1 state machine
		if (state1 == FingerState.FLICKING && timer1.seconds() >= FLICK_TIME_SECONDS)
		{
			finger1.setPosition(fingerConfig.getMinPosition());
			state1 = FingerState.IDLE;
		}

		// Finger 2
		if (state2 == FingerState.FLICKING && timer2.seconds() >= FLICK_TIME_SECONDS)
		{
			finger2.setPosition(fingerConfig.getMinPosition());
			state2 = FingerState.IDLE;
		}

		// Finger 3
		if (state3 == FingerState.FLICKING && timer3.seconds() >= FLICK_TIME_SECONDS)
		{
			finger3.setPosition(fingerConfig.getMinPosition());
			state3 = FingerState.IDLE;
		}

		// Optional debug
		 DebugUtil.logAdd("Finger positions: " + finger1.getPosition() + ", " + finger2.getPosition() + ", " + finger3.getPosition());
	}
}