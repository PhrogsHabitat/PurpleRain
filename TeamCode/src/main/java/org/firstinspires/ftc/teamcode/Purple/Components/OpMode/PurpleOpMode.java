package org.firstinspires.ftc.teamcode.Purple.Components.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class PurpleOpMode extends LinearOpMode
{
	@Override
	public void runOpMode ()
	{

		create();

		waitForStart();

		while (opModeIsActive())
		{
			update();
		}

		destroy();
	}

	public abstract void create ();

	public abstract void update ();

	public abstract void destroy ();
}