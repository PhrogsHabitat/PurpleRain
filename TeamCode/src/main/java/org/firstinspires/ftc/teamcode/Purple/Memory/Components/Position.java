package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Represents a position with x, y coordinates and heading.
 * Optionally backed by a goBILDA Pinpoint odometry computer.
 */
public class Position
{
	private static final String DEFAULT_HARDWARE_NAME = "odo";
	private static final double DEFAULT_X_OFFSET_MM = 0.0;
	private static final double DEFAULT_Y_OFFSET_MM = 0.0;
	private static final GoBildaPinpointDriver.GoBildaOdometryPods DEFAULT_POD = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
	private static final GoBildaPinpointDriver.EncoderDirection DEFAULT_FORWARD_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;
	private static final GoBildaPinpointDriver.EncoderDirection DEFAULT_STRAFE_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;

	private final GoBildaPinpointDriver odo;
	private double x;
	private double y;
	private double heading;
	private DistanceUnit distanceUnit = DistanceUnit.INCH;
	private AngleUnit angleUnit = AngleUnit.DEGREES;

	/**
	 * Constructs a position at (0, 0, 0) with no odometry backing.
	 */
	public Position ()
	{

		this.odo = null;
		this.x = 0;
		this.y = 0;
		this.heading = 0;
	}

	/**
	 * Constructs a position with specified coordinates and heading.
	 *
	 * @param x       The x coordinate.
	 * @param y       The y coordinate.
	 * @param heading The heading in degrees.
	 */
	public Position (double x, double y, double heading)
	{

		this.odo = null;
		this.x = x;
		this.y = y;
		this.heading = heading;
	}

	/**
	 * Constructs a Position that tracks the goBILDA Pinpoint odometry computer.
	 */
	public Position (HardwareMap hardwareMap)
	{

		this(hardwareMap, DEFAULT_HARDWARE_NAME, DEFAULT_X_OFFSET_MM, DEFAULT_Y_OFFSET_MM, DistanceUnit.MM,
				DEFAULT_POD, DEFAULT_FORWARD_DIR, DEFAULT_STRAFE_DIR);
	}

	/**
	 * Constructs a Position that tracks the goBILDA Pinpoint odometry computer with custom offsets.
	 */
	public Position (
			HardwareMap hardwareMap,
			String hardwareName,
			double xOffset,
			double yOffset,
			DistanceUnit offsetUnit)
	{

		this(hardwareMap, hardwareName, xOffset, yOffset, offsetUnit, DEFAULT_POD, DEFAULT_FORWARD_DIR, DEFAULT_STRAFE_DIR);
	}

	/**
	 * Constructs a Position that tracks the goBILDA Pinpoint odometry computer with full configuration.
	 */
	public Position (
			HardwareMap hardwareMap,
			String hardwareName,
			double xOffset,
			double yOffset,
			DistanceUnit offsetUnit,
			GoBildaPinpointDriver.GoBildaOdometryPods podType,
			GoBildaPinpointDriver.EncoderDirection forwardDirection,
			GoBildaPinpointDriver.EncoderDirection strafeDirection)
	{

		this.odo = hardwareMap.get(GoBildaPinpointDriver.class, hardwareName);

		odo.setOffsets(xOffset, yOffset, offsetUnit);
		odo.setEncoderResolution(podType);
		odo.setEncoderDirections(forwardDirection, strafeDirection);
		odo.resetPosAndIMU();
	}

	/**
	 * Updates the stored position from the odometry computer.
	 * Call this in your main loop.
	 */
	public void update ()
	{

		if (odo == null)
		{
			return;
		}

		odo.update();
		Pose2D pos = odo.getPosition();
		x = pos.getX(distanceUnit);
		y = pos.getY(distanceUnit);
		heading = pos.getHeading(angleUnit);
	}

	/**
	 * Sets the output units used by getters after update.
	 */
	public void setUnits (DistanceUnit distanceUnit, AngleUnit angleUnit)
	{

		this.distanceUnit = distanceUnit;
		this.angleUnit = angleUnit;
	}

	/**
	 * Gets the x coordinate.
	 *
	 * @return The x value.
	 */
	public double getX ()
	{

		return x;
	}

	/**
	 * Gets the y coordinate.
	 *
	 * @return The y value.
	 */
	public double getY ()
	{

		return y;
	}

	/**
	 * Gets the heading in degrees.
	 *
	 * @return The heading value.
	 */
	public double getHeading ()
	{

		return heading;
	}

}
