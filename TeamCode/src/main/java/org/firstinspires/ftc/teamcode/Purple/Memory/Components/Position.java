package org.firstinspires.ftc.teamcode.Purple.Memory.Components;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Purple.Components.Lime.LimeUtil;
import org.firstinspires.ftc.teamcode.Purple.Constants;

import java.util.List;

/**
 * Represents a position with x, y coordinates and heading.
 * Optionally backed by a goBILDA Pinpoint odometry computer.
 */
public class Position
{
	private static final long VISION_CORRECTION_MIN_INTERVAL_MS = 120;
	private static final long VISION_MAX_RESULT_STALENESS_MS = 100;
	private static final double VISION_CORRECTION_BLEND_ALPHA = 0.35;
	private static final double VISION_MAX_REFERENCE_DELTA_INCHES = 30.0;
	private static final int VISION_MIN_TAG_COUNT = 2;
	private static final double VISION_MIN_AVG_AREA = 0.15;
	private static final double VISION_MIN_SINGLE_TAG_AVG_AREA = 0.50;
	private static final double VISION_MAX_AVG_DIST_METERS = 4.0;
	private static final double VISION_MAX_STDDEV_X_METERS = 0.20;
	private static final double VISION_MAX_STDDEV_Y_METERS = 0.20;
	private static final double VISION_FIELD_MIN_INCHES = -8.0;
	private static final double VISION_FIELD_MAX_INCHES = 152.0;

	private static final String DEFAULT_HARDWARE_NAME = "odo";
	private static final double DEFAULT_X_OFFSET_MM = 0.0;
	private static final double DEFAULT_Y_OFFSET_MM = 0.0;
	private static final GoBildaPinpointDriver.GoBildaOdometryPods DEFAULT_POD = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
	private static final GoBildaPinpointDriver.EncoderDirection DEFAULT_FORWARD_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;
	private static final GoBildaPinpointDriver.EncoderDirection DEFAULT_STRAFE_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;

	private final GoBildaPinpointDriver odo;
	private final Follower follower;
	private double x;
	private double y;
	private double heading;
	private DistanceUnit distanceUnit = DistanceUnit.INCH;
	private AngleUnit angleUnit = AngleUnit.DEGREES;
	private Pose lastFollowerPose;
	private long lastVisionCorrectionMs = 0;

	/**
	 * Constructs a position at (0, 0, 0) with no odometry backing.
	 */
	public Position ()
	{

		this.odo = null;
		this.follower = null;
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
		this.follower = null;
		this.x = x;
		this.y = y;
		this.heading = heading;
	}

	/**
	 * Constructs a Position that tracks the Pedro follower pose.
	 */
	public Position (Follower follower)
	{

		this.odo = null;
		this.follower = follower;
		this.x = 0;
		this.y = 0;
		this.heading = 0;
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

		this.follower = null;
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

		if (follower != null)
		{
			updateFromFollower();
			maybeApplyVisionCorrection(LimeUtil.getResult());
			return;
		}

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

	private void updateFromFollower ()
	{

		Pose followerPose = follower.getPose();
		if (followerPose == null)
		{
			return;
		}

		if (lastFollowerPose == null)
		{
			x = followerPose.getX();
			y = followerPose.getY();
		}
		else
		{
			x += followerPose.getX() - lastFollowerPose.getX();
			y += followerPose.getY() - lastFollowerPose.getY();
		}

		heading = Math.toDegrees(followerPose.getHeading());
		lastFollowerPose = followerPose.copy();
	}

	private void maybeApplyVisionCorrection (LLResult result)
	{

		VisionPoseCandidate candidate = getVisionPoseCandidate(result);
		if (!passesVisionGate(result, candidate))
		{
			return;
		}

		Pose pedroPose = convertVisionPoseToPedro(candidate.pose3D);
		if (pedroPose == null)
		{
			return;
		}

		Pose referencePose = getVisionReferencePose();
		if (referencePose != null)
		{
			double correctionDeltaInches = distanceXY(referencePose, pedroPose);
			if (correctionDeltaInches > VISION_MAX_REFERENCE_DELTA_INCHES)
			{
				return;
			}
		}

		x += VISION_CORRECTION_BLEND_ALPHA * (pedroPose.getX() - x);
		y += VISION_CORRECTION_BLEND_ALPHA * (pedroPose.getY() - y);
		lastVisionCorrectionMs = System.currentTimeMillis();
	}

	private boolean passesVisionGate (LLResult result, VisionPoseCandidate candidate)
	{

		if (result == null || candidate == null || !result.isValid())
		{
			return false;
		}

		int tagCount = result.getBotposeTagCount();
		if (tagCount < 1)
		{
			return false;
		}

		List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
		if (fiducials == null || fiducials.isEmpty())
		{
			return false;
		}

		if (result.getStaleness() > VISION_MAX_RESULT_STALENESS_MS)
		{
			return false;
		}

		double avgArea = result.getBotposeAvgArea();
		if (avgArea < VISION_MIN_AVG_AREA)
		{
			return false;
		}

		if (tagCount < VISION_MIN_TAG_COUNT && avgArea < VISION_MIN_SINGLE_TAG_AVG_AREA)
		{
			return false;
		}

		double avgDist = result.getBotposeAvgDist();
		if (avgDist <= 0.0 || avgDist > VISION_MAX_AVG_DIST_METERS)
		{
			return false;
		}

		if (!isStddevWithinLimits(candidate.stddev))
		{
			return false;
		}

		long nowMs = System.currentTimeMillis();
		return nowMs - lastVisionCorrectionMs >= VISION_CORRECTION_MIN_INTERVAL_MS;
	}

	private boolean isStddevWithinLimits (double[] stddev)
	{

		if (!hasUsableStddev(stddev))
		{
			return false;
		}

		return stddev[0] <= VISION_MAX_STDDEV_X_METERS &&
				stddev[1] <= VISION_MAX_STDDEV_Y_METERS;
	}

	private VisionPoseCandidate getVisionPoseCandidate (LLResult result)
	{

		if (result == null)
		{
			return null;
		}

		double[] mt2Stddev = result.getStddevMt2();
		if (hasUsableStddev(mt2Stddev))
		{
			return new VisionPoseCandidate(result.getBotpose_MT2(), mt2Stddev);
		}

		double[] mt1Stddev = result.getStddevMt1();
		if (hasUsableStddev(mt1Stddev))
		{
			return new VisionPoseCandidate(result.getBotpose(), mt1Stddev);
		}

		return null;
	}

	private boolean hasUsableStddev (double[] stddev)
	{

		if (stddev == null || stddev.length < 2)
		{
			return false;
		}

		if (Double.isNaN(stddev[0]) || Double.isNaN(stddev[1]) ||
				Double.isInfinite(stddev[0]) || Double.isInfinite(stddev[1]))
		{
			return false;
		}

		// Limelight reports zeros when stdev is unavailable in some pipelines.
		return stddev[0] > 0.0 || stddev[1] > 0.0;
	}

	private Pose convertVisionPoseToPedro (Pose3D pose3D)
	{

		if (pose3D == null || pose3D.getPosition() == null)
		{
			return null;
		}

		org.firstinspires.ftc.robotcore.external.navigation.Position visionPosition = pose3D.getPosition();
		double xInches = DistanceUnit.INCH.fromUnit(visionPosition.unit, visionPosition.x);
		double yInches = DistanceUnit.INCH.fromUnit(visionPosition.unit, visionPosition.y);

		double headingRadians = 0.0;
		if (pose3D.getOrientation() != null)
		{
			headingRadians = pose3D.getOrientation().getYaw(AngleUnit.RADIANS);
		}

		Pose nativePedroPose = new Pose(xInches, yInches, headingRadians);
		Pose ftcPose = new Pose(xInches, yInches, headingRadians, FTCCoordinates.INSTANCE);
		Pose convertedFtcPose = FTCCoordinates.INSTANCE.convertToPedro(ftcPose);
		Pose selectedPose = choosePedroPose(nativePedroPose, convertedFtcPose);
		return applyTurretMountTranslationCorrection(selectedPose);
	}

	private Pose choosePedroPose (Pose nativePedroPose, Pose convertedFtcPose)
	{

		boolean nativeInBounds = isLikelyFieldPose(nativePedroPose);
		boolean convertedInBounds = isLikelyFieldPose(convertedFtcPose);

		if (nativeInBounds && !convertedInBounds)
		{
			return nativePedroPose;
		}

		if (convertedInBounds && !nativeInBounds)
		{
			return convertedFtcPose;
		}

		Pose referencePose = getVisionReferencePose();
		if (referencePose == null)
		{
			// Preserve prior behavior when there is no localization context.
			return convertedFtcPose;
		}

		double nativeDistance = distanceXY(referencePose, nativePedroPose);
		double convertedDistance = distanceXY(referencePose, convertedFtcPose);
		return nativeDistance <= convertedDistance ? nativePedroPose : convertedFtcPose;
	}

	private Pose getVisionReferencePose ()
	{

		if (lastFollowerPose != null)
		{
			return lastFollowerPose;
		}

		return new Pose(x, y, Math.toRadians(heading));
	}

	private boolean isLikelyFieldPose (Pose pose)
	{

		if (pose == null)
		{
			return false;
		}

		return pose.getX() >= VISION_FIELD_MIN_INCHES &&
				pose.getX() <= VISION_FIELD_MAX_INCHES &&
				pose.getY() >= VISION_FIELD_MIN_INCHES &&
				pose.getY() <= VISION_FIELD_MAX_INCHES;
	}

	private double distanceXY (Pose a, Pose b)
	{

		double deltaX = a.getX() - b.getX();
		double deltaY = a.getY() - b.getY();
		return Math.hypot(deltaX, deltaY);
	}

	private Pose applyTurretMountTranslationCorrection (Pose pose)
	{

		if (pose == null || !Constants.LIMELIGHT_DYNAMIC_MOUNT_COMPENSATION)
		{
			return pose;
		}

		double turretDeltaDeg = LimeUtil.getTurretYawDeltaDegrees();
		if (Math.abs(turretDeltaDeg) < 1e-6)
		{
			return pose;
		}

		double configuredForwardInches = DistanceUnit.INCH.fromUnit(DistanceUnit.METER, Constants.LIMELIGHT_FORWARD_METERS);
		double configuredRightInches = DistanceUnit.INCH.fromUnit(DistanceUnit.METER, Constants.LIMELIGHT_RIGHT_METERS);
		double turretDeltaRad = Math.toRadians(turretDeltaDeg);

		// Rotate the configured camera offset by the turret angle to estimate actual offset.
		double dynamicForwardInches = (configuredForwardInches * Math.cos(turretDeltaRad)) +
				(configuredRightInches * Math.sin(turretDeltaRad));
		double dynamicRightInches = (-configuredForwardInches * Math.sin(turretDeltaRad)) +
				(configuredRightInches * Math.cos(turretDeltaRad));

		double correctionForwardInches = configuredForwardInches - dynamicForwardInches;
		double correctionRightInches = configuredRightInches - dynamicRightInches;

		double headingRad = Math.toRadians(heading);
		double correctionX = (correctionForwardInches * Math.cos(headingRad)) +
				(correctionRightInches * Math.sin(headingRad));
		double correctionY = (correctionForwardInches * Math.sin(headingRad)) -
				(correctionRightInches * Math.cos(headingRad));

		return new Pose(
				pose.getX() + correctionX,
				pose.getY() + correctionY,
				pose.getHeading()
		);
	}

	private static final class VisionPoseCandidate
	{
		private final Pose3D pose3D;
		private final double[] stddev;

		private VisionPoseCandidate (Pose3D pose3D, double[] stddev)
		{

			this.pose3D = pose3D;
			this.stddev = stddev;
		}
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
