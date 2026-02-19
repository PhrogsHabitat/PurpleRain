//package org.firstinspires.ftc.teamcode.Purple.Memory.Components.Reference;
//
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.helpers.ColorConverter;
//import org.firstinspires.ftc.teamcode.helpers.Config;
//import org.firstinspires.ftc.teamcode.helpers.Kolor;
//
//public class ColorSensor
//{
//
//	/**
//	 * The color sensor
//	 */
//	private final RevColorSensorV3 sensor;
//
//	/**
//	 *
//	 * @param hardwareMap Hardware map for robot
//	 * @param deviceName  Device name in config
//	 */
//	public ColorSensor (HardwareMap hardwareMap, String deviceName)
//	{
//
//		sensor = hardwareMap.get(RevColorSensorV3.class, deviceName);
//		sensor.setGain(6); //default 1.0
//	}
//
//	/**
//	 *
//	 * @param unit The unit of measure
//	 * @return The distance to object
//	 */
//	public double getDistance (DistanceUnit unit)
//	{
//
//		return sensor.getDistance(unit);
//	}
//
//	/**
//	 * Processes the colors and distance to determine artifact color
//	 *
//	 * @return boolean True if color found
//	 */
//	public boolean hasArtifact ()
//	{
//
//		return getDistance(DistanceUnit.MM) < Config.ARTIFACT_MAX_DISTANCE;
//	}
//
//	/**
//	 * Gets the artifact color that was found by hasArtifact
//	 *
//	 * @return String color
//	 */
//	public Kolor getCurrentColor ()
//	{
//
//		NormalizedRGBA colors = sensor.getNormalizedColors();
//		// Get the double values by calling the deltaRgb method
//		double dg = deltaRgb(Config.ARTIFACT_GREEN_RGB, colors);
//		double dp = deltaRgb(Config.ARTIFACT_PURPLE_RGB, colors);
//		double db = deltaRgb(Config.ARTIFACT_BLACK_RGB, colors);
//
//		// Find the minimum value
//		double smallestDelta = dg;
//		Kolor smallestColor = Kolor.GREEN;
//
//		if (dp < smallestDelta)
//		{
//			smallestDelta = dp;
//			smallestColor = Kolor.PURPLE;
//		}
//		if (db < smallestDelta)
//		{
//			smallestColor = Kolor.BLACK;
//		}
//
//		return smallestColor;
//	}
//
//	/**
//	 * Prints color data
//	 *
//	 * @param telemetry object to print to
//	 */
//	public void print (Telemetry telemetry)
//	{
//
//		NormalizedRGBA colors = sensor.getNormalizedColors();
//
//		int r = (int) (colors.red * 1000);
//		int g = (int) (colors.green * 1000);
//		int b = (int) (colors.blue * 1000);
//
//		double dg = deltaRgb(Config.ARTIFACT_GREEN_RGB, colors);
//		double dp = deltaRgb(Config.ARTIFACT_PURPLE_RGB, colors);
//		double dy = deltaRgb(Config.ARTIFACT_YELLOW_RGB, colors);
//		double db = deltaRgb(Config.ARTIFACT_BLACK_RGB, colors);
//
//		telemetry.addData("Red", r);
//		telemetry.addData("Green", g);
//		telemetry.addData("Blue", b);
//		telemetry.addData("DG", "%.3f", dg);
//		telemetry.addData("DP", "%.3f", dp);
//		telemetry.addData("DY", "%.3f", dy);
//		telemetry.addData("DB", "%.3f", db);
//		telemetry.addData("Prox", "%.3f", sensor.getDistance(DistanceUnit.MM));
//	}
//
//	/**
//	 * Get delta from color
//	 *
//	 * @param match array of rgb to match
//	 * @return arbitrary number distance from match
//	 */
//	private double deltaRgb (double[] match, NormalizedRGBA colors)
//	{
//
//		int r = (int) (colors.red * 1000);
//		int g = (int) (colors.green * 1000);
//		int b = (int) (colors.blue * 1000);
//		double[] lab = ColorConverter.rgbToLab(r, g, b);
//
//		int x = (int) (match[0] * 1000);
//		int y = (int) (match[1] * 1000);
//		int z = (int) (match[2] * 1000);
//		double[] pqz = ColorConverter.rgbToLab(x, y, z);
//
//		//test
//		double rx = Math.pow(pqz[0] - lab[0], 2);
//		double gx = Math.pow(pqz[1] - lab[1], 2);
//		double bx = Math.pow(pqz[2] - lab[2], 2);
//		double t = (pqz[0] + lab[0]) / 2;
//		return Math.sqrt(2 * rx + 4 * gx + 3 * bx + t * (rx - bx) / 256);
//	}
//}
