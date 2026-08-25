// package org.firstinspires.ftc.teamcode.Purple.Components.Motors;

// import com.arcrobotics.ftclib.hardware.motors.Motor;
// import com.arcrobotics.ftclib.hardware.motors.MotorEx;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.HardwareMap;

// public final class MotorConfig
// {
// 	private final String name;
// 	private final Position configuredPosition;
// 	private final MotorEx motor;
// 	private final double maxRPM;
// 	private final double cpr;
// 	private double ticksPerRevolution;

// 	private boolean inverted;
// 	private boolean runWithEncoder;
// 	private Motor.ZeroPowerBehavior zeroPowerBehavior;
// 	private ControlMode controlMode;
// 	private double velocitySign = 1.0;

// 	private double targetRPM;
// 	private double targetVelocityTicksPerSecond;
// 	private int targetPositionTicks;

// 	private MotorConfig (Builder b)
// 	{
// 		this.name = b.name;
// 		this.configuredPosition = b.position;
// 		this.maxRPM = b.maxRPM;
// 		this.cpr = b.cpr;
// 		this.ticksPerRevolution = b.ticksPerRevolution > 0 ? b.ticksPerRevolution : b.cpr;
// 		this.inverted = b.inverted;
// 		this.zeroPowerBehavior = b.zeroPowerBehavior;
// 		this.controlMode = b.controlMode;
// 		this.runWithEncoder = b.runWithEncoder;

// 		if (b.cpr > 0 && b.maxRPM > 0)
// 		{
// 			this.motor = new MotorEx(b.hardwareMap, b.name, b.cpr, b.maxRPM);
// 		}
// 		else
// 		{
// 			this.motor = new MotorEx(b.hardwareMap, b.name);
// 		}

// 		motor.setInverted(inverted);
// 		motor.setZeroPowerBehavior(zeroPowerBehavior);
// 		setRunWithEncoder(runWithEncoder);
// 		setControlMode(controlMode);

// 		if (b.velocityPid != null)
// 		{
// 			setVelocityCoefficients(b.velocityPid[0], b.velocityPid[1], b.velocityPid[2]);
// 		}
// 		if (b.positionCoefficient != null)
// 		{
// 			setPositionCoefficient(b.positionCoefficient);
// 		}
// 		if (b.positionTolerance != null)
// 		{
// 			setPositionTolerance(b.positionTolerance);
// 		}
// 		if (b.feedforwardTwo != null)
// 		{
// 			setFeedforwardCoefficients(b.feedforwardTwo[0], b.feedforwardTwo[1]);
// 		}
// 		if (b.feedforwardThree != null)
// 		{
// 			setFeedforwardCoefficients(b.feedforwardThree[0], b.feedforwardThree[1], b.feedforwardThree[2]);
// 		}
// 	}

// 	private static double clamp (double value, double min, double max)
// 	{
// 		return Math.max(min, Math.min(max, value));
// 	}

// 	private void requireTicksPerRevolution ()
// 	{
// 		if (ticksPerRevolution <= 0)
// 		{
// 			throw new IllegalStateException("ticksPerRevolution must be > 0 to use RPM methods.");
// 		}
// 	}

// 	private double rpmToTicksPerSecond (double rpm)
// 	{
// 		return rpm * ticksPerRevolution / 60.0;
// 	}

// 	private double ticksPerSecondToRpm (double ticksPerSecond)
// 	{
// 		return ticksPerSecond * 60.0 / ticksPerRevolution;
// 	}

// 	private double toMotorVelocityFrame (double ticksPerSecond)
// 	{
// 		return inverted ? -ticksPerSecond : ticksPerSecond;
// 	}

// 	private double fromMotorVelocityFrame (double ticksPerSecond)
// 	{
// 		return inverted ? -ticksPerSecond : ticksPerSecond;
// 	}

// 	private double toEncoderVelocityFrame (double ticksPerSecond)
// 	{
// 		return ticksPerSecond * velocitySign;
// 	}

// 	private double fromEncoderVelocityFrame (double ticksPerSecond)
// 	{
// 		return ticksPerSecond / velocitySign;
// 	}

// 	public String getName ()
// 	{
// 		return name;
// 	}

// 	public Position getConfiguredPosition ()
// 	{
// 		return configuredPosition;
// 	}

// 	public double getMaxRPM ()
// 	{
// 		return maxRPM;
// 	}

// 	public double getCPR ()
// 	{
// 		return cpr;
// 	}

// 	public ControlMode getControlMode ()
// 	{
// 		return controlMode;
// 	}

// 	public void setControlMode (ControlMode mode)
// 	{
// 		if (mode == null)
// 		{
// 			throw new IllegalArgumentException("ControlMode cannot be null.");
// 		}

// 		if ((mode == ControlMode.VELOCITY_CONTROL || mode == ControlMode.POSITION_CONTROL) && !runWithEncoder)
// 		{
// 			setRunWithEncoder(true);
// 		}

// 		this.controlMode = mode;
// 		switch (mode)
// 		{
// 			case VELOCITY_CONTROL:
// 				motor.setRunMode(Motor.RunMode.VelocityControl);
// 				break;
// 			case POSITION_CONTROL:
// 				motor.setRunMode(Motor.RunMode.PositionControl);
// 				break;
// 			case RAW_POWER:
// 			default:
// 				motor.setRunMode(Motor.RunMode.RawPower);
// 				break;
// 		}
// 	}

// 	public boolean isRunWithEncoder ()
// 	{
// 		return runWithEncoder;
// 	}

// 	public void setRunWithEncoder (boolean enabled)
// 	{
// 		runWithEncoder = enabled;
// 		motor.motorEx.setMode(enabled ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	}

// 	public boolean isInverted ()
// 	{
// 		return inverted;
// 	}

// 	public void setInverted (boolean isInverted)
// 	{
// 		inverted = isInverted;
// 		motor.setInverted(isInverted);
// 	}

// 	public Motor.ZeroPowerBehavior getZeroPowerBehavior ()
// 	{
// 		return zeroPowerBehavior;
// 	}

// 	public void setZeroPowerBehavior (Motor.ZeroPowerBehavior behavior)
// 	{
// 		if (behavior == null)
// 		{
// 			throw new IllegalArgumentException("ZeroPowerBehavior cannot be null.");
// 		}
// 		zeroPowerBehavior = behavior;
// 		motor.setZeroPowerBehavior(behavior);
// 	}

// 	public void setTicksPerRevolution (double ticksPerRevolution)
// 	{
// 		if (ticksPerRevolution <= 0)
// 		{
// 			throw new IllegalArgumentException("ticksPerRevolution must be > 0.");
// 		}
// 		this.ticksPerRevolution = ticksPerRevolution;
// 	}

// 	public double getTicksPerRevolution ()
// 	{
// 		return ticksPerRevolution;
// 	}

// 	public void setVelocityDirectionReversed (boolean reversed)
// 	{
// 		velocitySign = reversed ? -1.0 : 1.0;
// 	}

// 	public boolean isVelocityDirectionReversed ()
// 	{
// 		return velocitySign < 0;
// 	}

// 	public void setPower (double power)
// 	{
// 		motor.motorEx.setPower(clamp(power, -1.0, 1.0));
// 		targetVelocityTicksPerSecond = 0;
// 		targetRPM = 0;
// 	}

// 	public double getPower ()
// 	{
// 		return motor.get();
// 	}

// 	public void setVelocity (double ticksPerSecond)
// 	{
// 		if (controlMode != ControlMode.VELOCITY_CONTROL)
// 		{
// 			throw new IllegalStateException("setVelocity requires ControlMode.VELOCITY_CONTROL.");
// 		}
// 		targetVelocityTicksPerSecond = ticksPerSecond;
// 		if (ticksPerRevolution > 0)
// 		{
// 			targetRPM = ticksPerSecondToRpm(ticksPerSecond);
// 		}
// 		motor.setVelocity(toMotorVelocityFrame(toEncoderVelocityFrame(ticksPerSecond)));
// 	}

// 	public double getVelocity ()
// 	{
// 		return fromEncoderVelocityFrame(fromMotorVelocityFrame(motor.getVelocity()));
// 	}

// 	public double getTargetVelocity ()
// 	{
// 		return targetVelocityTicksPerSecond;
// 	}

// 	public void setTargetPosition (int positionTicks)
// 	{
// 		if (controlMode != ControlMode.POSITION_CONTROL)
// 		{
// 			throw new IllegalStateException("setTargetPosition requires ControlMode.POSITION_CONTROL.");
// 		}
// 		targetPositionTicks = positionTicks;
// 		motor.setTargetPosition(positionTicks);
// 	}

// 	public int getTargetPosition ()
// 	{
// 		return targetPositionTicks;
// 	}

// 	public int getPosition ()
// 	{
// 		return motor.getCurrentPosition();
// 	}

// 	public void resetEncoder ()
// 	{
// 		motor.stopAndResetEncoder();
// 		if (!runWithEncoder)
// 		{
// 			motor.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 		}
// 	}

// 	public void setVelocityCoefficients (double kp, double ki, double kd)
// 	{
// 		motor.setVeloCoefficients(kp, ki, kd);
// 	}

// 	public void setPositionCoefficient (double kp)
// 	{
// 		motor.setPositionCoefficient(kp);
// 	}

// 	public void setPositionTolerance (double tolerance)
// 	{
// 		motor.setPositionTolerance(tolerance);
// 	}

// 	public void setFeedforwardCoefficients (double ks, double kv)
// 	{
// 		motor.setFeedforwardCoefficients(ks, kv);
// 	}

// 	public void setFeedforwardCoefficients (double ks, double kv, double ka)
// 	{
// 		motor.setFeedforwardCoefficients(ks, kv, ka);
// 	}

// 	public void setRPM (double rpm)
// 	{
// 		requireTicksPerRevolution();
// 		targetRPM = rpm;
// 		setVelocity(rpmToTicksPerSecond(rpm));
// 	}

// 	public double getTargetRPM ()
// 	{
// 		return targetRPM;
// 	}

// 	public double getRPM ()
// 	{
// 		if (ticksPerRevolution <= 0)
// 		{
// 			return 0;
// 		}
// 		return ticksPerSecondToRpm(getVelocity());
// 	}

// 	public void stop ()
// 	{
// 		motor.stopMotor();
// 		targetRPM = 0;
// 		targetVelocityTicksPerSecond = 0;
// 	}

// 	public void update ()
// 	{
// 	}

// 	public enum Position
// 	{
// 		FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, EXPLOSHER, INTAKE, MIDTAKE
// 	}

// 	public enum ControlMode
// 	{
// 		RAW_POWER, VELOCITY_CONTROL, POSITION_CONTROL
// 	}

// 	public static class Builder
// 	{
// 		private final HardwareMap hardwareMap;
// 		private final String name;
// 		private final Position position;
// 		private final double maxRPM;
// 		private final double cpr;

// 		private boolean inverted = false;
// 		private Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
// 		private ControlMode controlMode;
// 		private boolean runWithEncoder;
// 		private double ticksPerRevolution;

// 		private double[] velocityPid;
// 		private Double positionCoefficient;
// 		private Double positionTolerance;
// 		private double[] feedforwardTwo;
// 		private double[] feedforwardThree;

// 		public Builder (HardwareMap hw, String name, Position position, double cpr, double maxRPM)
// 		{
// 			this.hardwareMap = hw;
// 			this.name = name;
// 			this.position = position;
// 			this.cpr = cpr;
// 			this.maxRPM = maxRPM;
// 			this.ticksPerRevolution = cpr;
// 			this.controlMode = cpr > 0 && maxRPM > 0 ? ControlMode.VELOCITY_CONTROL : ControlMode.RAW_POWER;
// 			this.runWithEncoder = cpr > 0;
// 		}

// 		public Builder (HardwareMap hw, String name, Position position)
// 		{
// 			this(hw, name, position, 0, 0);
// 		}

// 		public Builder inverted ()
// 		{
// 			this.inverted = true;
// 			return this;
// 		}

// 		public Builder zeroPowerBehavior (Motor.ZeroPowerBehavior zeroPowerBehavior)
// 		{
// 			this.zeroPowerBehavior = zeroPowerBehavior;
// 			return this;
// 		}

// 		public Builder disableVelocityControl ()
// 		{
// 			this.controlMode = ControlMode.RAW_POWER;
// 			this.runWithEncoder = false;
// 			return this;
// 		}

// 		public Builder controlMode (ControlMode mode)
// 		{
// 			this.controlMode = mode;
// 			if ((mode == ControlMode.VELOCITY_CONTROL || mode == ControlMode.POSITION_CONTROL) && !runWithEncoder)
// 			{
// 				this.runWithEncoder = true;
// 			}
// 			return this;
// 		}

// 		public Builder runWithEncoder (boolean runWithEncoder)
// 		{
// 			this.runWithEncoder = runWithEncoder;
// 			return this;
// 		}

// 		public Builder ticksPerRevolution (double ticksPerRevolution)
// 		{
// 			if (ticksPerRevolution <= 0)
// 			{
// 				throw new IllegalArgumentException("ticksPerRevolution must be > 0.");
// 			}
// 			this.ticksPerRevolution = ticksPerRevolution;
// 			return this;
// 		}

// 		public Builder velocityCoefficients (double kp, double ki, double kd)
// 		{
// 			this.velocityPid = new double[]{kp, ki, kd};
// 			return this;
// 		}

// 		public Builder positionCoefficient (double kp)
// 		{
// 			this.positionCoefficient = kp;
// 			return this;
// 		}

// 		public Builder positionTolerance (double tolerance)
// 		{
// 			this.positionTolerance = tolerance;
// 			return this;
// 		}

// 		public Builder feedforwardCoefficients (double ks, double kv)
// 		{
// 			this.feedforwardTwo = new double[]{ks, kv};
// 			this.feedforwardThree = null;
// 			return this;
// 		}

// 		public Builder feedforwardCoefficients (double ks, double kv, double ka)
// 		{
// 			this.feedforwardThree = new double[]{ks, kv, ka};
// 			this.feedforwardTwo = null;
// 			return this;
// 		}

// 		public MotorConfig build ()
// 		{
// 			return new MotorConfig(this);
// 		}
// 	}
// }