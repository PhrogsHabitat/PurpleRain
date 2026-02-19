package org.firstinspires.ftc.teamcode.Purple.Memory.Components.Reference;

public class Config
{

	//obelisk april tag numbers
	public static final int PGP = 22;
	public static final int GPP = 21;
	public static final int PPG = 23;
	//weight of robot
	public static double WEIGHT = 23.6 / 2.205; //13.2 lbs to kg
	public static String LEFT_OUTER_COLOR_SENSOR = "leftOuterColorSensor";
	public static String LEFT_INNER_COLOR_SENSOR = "leftInnerColorSensor";
	public static String CENTER_LEFT_COLOR_SENSOR = "centerLeftColorSensor";
	public static String CENTER_RIGHT_COLOR_SENSOR = "centerRightColorSensor";
	public static String RIGHT_OUTER_COLOR_SENSOR = "rightOuterColorSensor";
	public static String RIGHT_INNER_COLOR_SENSOR = "rightInnerColorSensor";
	public static String FRONT_LEFT = "frontLeft";
	public static String FRONT_RIGHT = "frontRight";
	public static String BACK_LEFT = "backLeft";
	public static String BACK_RIGHT = "backRight";
	public static String LEFT_LIGHT = "leftLight";
	public static String CENTER_LIGHT = "centerLight";
	public static String RIGHT_LIGHT = "rightLight";
	public static String ODO = "odo";
	public static String MIXER = "mixer";
	public static String FEEDBACK = "feedback";
	public static String HOOD_LEFT = "hoodLeft";
	public static String HOOD_RIGHT = "hoodRight";
	public static String KICKER = "kicker";
	public static String INTAKE = "intake";
	public static String SHOOTER = "shooter";
	public static String SHOOTER_ENCODER = "shooterEncoder";
	public static String LIMELIGHT = "limelight";
	public static String SECOND_SHOOTER = "shooter2";
	//color sensor distance from artifact (mm)
	public static int ARTIFACT_MAX_DISTANCE = 14;
	//artifact rgb colors
	public static double[] ARTIFACT_PURPLE_RGB = {.050, .025, .060};
	public static double[] ARTIFACT_GREEN_RGB = {.024, .071, .030};
	public static double[] ARTIFACT_YELLOW_RGB = {.255, .255, .000};
	public static double[] ARTIFACT_BLACK_RGB = {.000, .000, .000};
	public static double INTAKE_POWER = 0.84;
	public static double GREEN_LIGHT = 0.500;
	public static double PURPLE_LIGHT = 0.720;

	public static double HOOD_AUDIENCE = 0.47;
	public static double HOOD_GOAL = 0.08;

	//limelight height from floor (inches)
	public static double LIMELIGHT_HEIGHT = 15.5;

	//limelight angle relative to floor, positive is pointed up
	public static double LIMELIGHT_VERTICAL_ANGLE = 12.45;

	//odometry
	public static double ODO_X_MM = 76.2;
	public static double ODO_Y_MM = -130.175;

	//persistence file names
	public static String POSITION_FILE = "position.txt";
	public static String MIXER_FILE = "mixer.txt";
	public static String SHOOTER_FILE = "shooter.txt";
	public static String KICKER_FILE = "kicker.txt";
	public static String AUTO_FILE = "auto.txt";

}
