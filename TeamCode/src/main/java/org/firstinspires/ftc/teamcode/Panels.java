package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public final class Panels
{
	private static final double ROBOT_RADIUS = 9.0;
	private static final FieldManager FIELD = PanelsField.INSTANCE.getField();
	private static final Style DEFAULT_ROBOT_STYLE = new Style("", "#3F51B5", 0.75);

	private Panels ()
	{
	}

	public static TelemetryManager telemetry ()
	{
		return PanelsTelemetry.INSTANCE.getTelemetry();
	}

	public static void initPedroField ()
	{
		FIELD.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
	}

	public static void drawRobot (Pose pose)
	{
		drawRobot(pose, DEFAULT_ROBOT_STYLE);
	}

	public static void drawRobot (Pose pose, Style style)
	{
		if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading()))
		{
			return;
		}

		FIELD.setStyle(style);
		FIELD.moveCursor(pose.getX(), pose.getY());
		FIELD.circle(ROBOT_RADIUS);

		Vector headingVector = pose.getHeadingAsUnitVector();
		headingVector.setMagnitude(headingVector.getMagnitude() * ROBOT_RADIUS);

		double x1 = pose.getX() + headingVector.getXComponent() / 2.0;
		double y1 = pose.getY() + headingVector.getYComponent() / 2.0;
		double x2 = pose.getX() + headingVector.getXComponent();
		double y2 = pose.getY() + headingVector.getYComponent();

		FIELD.moveCursor(x1, y1);
		FIELD.line(x2, y2);
	}

	public static void send ()
	{
		FIELD.update();
	}
}
