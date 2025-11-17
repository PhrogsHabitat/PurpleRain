package org.firstinspires.ftc.teamcode.Purple.Components.Motors;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class MotorConfig
{

	private final String name;
	private final Position position;
	private final Motor motor;
	// --- PID coefficients ---
	private final double veloP = 1.0; // Low value, I gotta tune this
	private final double veloI = 0; // Very low value, I gotta tune this
	private final double veloD = 0; // Low value, I gotta tune this
	private final double positionP = 1.0;
	private MotorState state = MotorState.OFF;
	private ControlMode currentControlMode = ControlMode.VELOCITY_CONTROL;
	private double targetRPM = 0;

	private MotorConfig (Builder builder)
	{
		this.name = builder.name;
		this.position = builder.position;

		if (builder.useMotorEx)
		{
			this.motor = new MotorEx(builder.hardwareMap, builder.name);
		} else if (builder.maxRPM != 0 && builder.cpr != 0)
		{
			this.motor = new Motor(builder.hardwareMap, builder.name, builder.cpr, builder.maxRPM);
		} else
		{
			this.motor = new Motor(builder.hardwareMap, builder.name);
		}

		motor.setInverted(builder.inverted);
		motor.setZeroPowerBehavior(builder.zeroPowerBehavior);

		initializePIDCoefficients();
	}

	private void initializePIDCoefficients ()
	{
		motor.setVeloCoefficients(veloP, veloI, veloD);
		motor.setPositionCoefficient(positionP);
		motor.setFeedforwardCoefficients(5, 5);
	}

	private double rpmToTicksPerSecond (double rpm)
	{
		return (rpm * motor.getCPR()) / 60.0;
	}

	private double ticksPerSecondToRpm (double tps)
	{
		return (tps * 60.0) / motor.getCPR();
	}

	// ***************** BUILDER *****************

	public double getCurrentRPM ()
	{
		return ticksPerSecondToRpm(motor.getCorrectedVelocity());
	}

	// ***************** INTERNAL HELPERS *****************

	public double getTargetRPM ()
	{
		return targetRPM;
	}

	public void setTargetRPM (double output)
	{

		// Ugh. (FNF Reference)


		// We will stop the motor and reset encoder for now to see what happens
		// motor.stopAndResetEncoder();

		// Then set the control mode once again, just to be safe
		setControlMode(ControlMode.VELOCITY_CONTROL);
		this.targetRPM = output;

		// Then do the unsafe stuff
		motor.set(output);

		state = (output != 0) ? MotorState.ON : MotorState.OFF;
	}

	public void setPower (double power)
	{
		setControlMode(ControlMode.RAW_POWER);
		motor.set(power);
		state = (power != 0) ? MotorState.ON : MotorState.OFF;
	}

	// ***************** VELOCITY CONTROL *****************

	public void stop ()
	{
		motor.stopMotor();
		state = MotorState.OFF;
	}

	public String getName ()
	{
		return name;
	}

	public Position getPosition ()
	{
		return position;
	}

	// ***************** CONTROL MODE *****************

	public MotorState getState ()
	{
		return state;
	}

	// ***************** RAW POWER CONTROL *****************

	public void setState (MotorState state)
	{
		this.state = state;
		if (state == MotorState.OFF) stop();
	}

	public ControlMode getControlMode ()
	{
		return currentControlMode;
	}

	// ***************** GETTERS *****************

	public void setControlMode (ControlMode controlMode)
	{
		this.currentControlMode = controlMode;

		switch (controlMode)
		{
			case VELOCITY_CONTROL:
				motor.setRunMode(Motor.RunMode.VelocityControl);
				break;

			case POSITION_CONTROL:
				motor.setRunMode(Motor.RunMode.PositionControl);
				break;

			case RAW_POWER:
			default:
				motor.setRunMode(Motor.RunMode.RawPower);
				break;
		}
	}

	public enum Position
	{
		FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, EXPLOSHER, INTAKE, MIDTAKE
	}

	public enum MotorState
	{
		ON, OFF
	}

	public enum ControlMode
	{
		RAW_POWER, VELOCITY_CONTROL, POSITION_CONTROL
	}

	public static class Builder
	{
		private final HardwareMap hardwareMap;
		private final String name;
		private final Position position;
		private final double maxRPM;
		private final double cpr;

		private boolean inverted = false;
		private Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
		private boolean useMotorEx = false;

		public Builder (HardwareMap hardwareMap, String name, Position position, double cpr, double maxRPM)
		{
			this.hardwareMap = hardwareMap;
			this.name = name;
			this.position = position;
			this.maxRPM = maxRPM;
			this.cpr = cpr;
		}

		public Builder (HardwareMap hardwareMap, String name, Position position)
		{
			this(hardwareMap, name, position, 0, 0);
		}

		public Builder inverted ()
		{
			this.inverted = true;
			return this;
		}

		public Builder zeroPowerBehavior (Motor.ZeroPowerBehavior zeroPowerBehavior)
		{
			this.zeroPowerBehavior = zeroPowerBehavior;
			return this;
		}

		public Builder useMotorEx ()
		{
			this.useMotorEx = true;
			return this;
		}

		public MotorConfig build ()
		{
			return new MotorConfig(this);
		}
	}
}
