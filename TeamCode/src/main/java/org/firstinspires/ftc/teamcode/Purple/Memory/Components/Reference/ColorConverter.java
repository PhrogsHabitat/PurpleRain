package org.firstinspires.ftc.teamcode.Purple.Memory.Components.Reference;

public class ColorConverter
{
	// Reference white point D65
	private static final double D65_X = 95.047;
	private static final double D65_Y = 100.000;
	private static final double D65_Z = 108.883;

	public static double[] rgbToLab (int r, int g, int b)
	{
		// 1. Normalize RGB values to 0-1
		double R = r / 255.0;
		double G = g / 255.0;
		double B = b / 255.0;

		// 2. Convert sRGB to XYZ (with gamma correction)
		R = (R > 0.04045) ? Math.pow((R + 0.055) / 1.055, 2.4) : (R / 12.92);
		G = (G > 0.04045) ? Math.pow((G + 0.055) / 1.055, 2.4) : (G / 12.92);
		B = (B > 0.04045) ? Math.pow((B + 0.055) / 1.055, 2.4) : (B / 12.92);

		// Linear RGB to XYZ conversion matrix
		double X = (R * 0.4124 + G * 0.3576 + B * 0.1805) * 100;
		double Y = (R * 0.2126 + G * 0.7152 + B * 0.0722) * 100;
		double Z = (R * 0.0193 + G * 0.1192 + B * 0.9505) * 100;

		// 3. Convert XYZ to CIELAB
		double x_norm = X / D65_X;
		double y_norm = Y / D65_Y;
		double z_norm = Z / D65_Z;

		x_norm = (x_norm > 0.008856) ? Math.cbrt(x_norm) : (7.787 * x_norm + 16.0 / 116.0);
		y_norm = (y_norm > 0.008856) ? Math.cbrt(y_norm) : (7.787 * y_norm + 16.0 / 116.0);
		z_norm = (z_norm > 0.008856) ? Math.cbrt(z_norm) : (7.787 * z_norm + 16.0 / 116.0);

		double L = (116 * y_norm) - 16;
		double a = 500 * (x_norm - y_norm);
		double m = 200 * (y_norm - z_norm);

		return new double[]{L, a, m};
	}
}
