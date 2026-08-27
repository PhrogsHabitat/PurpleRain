package org.firstinspires.ftc.teamcode.Purple.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Gamepad Diagnostic", group = "Tools")
public class GamepadDiagnosticOpMode extends LinearOpMode
{
	@Override
	public void runOpMode () throws InterruptedException
	{
		telemetry.addLine("Press Start to scan Gamepad1");
		telemetry.update();

		waitForStart();

		while (opModeIsActive())
		{
			Class<?> clazz = gamepad1.getClass();
			telemetry.addData("Class", clazz.getName());
			
			List<String> methods = new ArrayList<>();
			for (Method m : clazz.getMethods()) {
				methods.add(m.getName());
			}
			
			List<String> fields = new ArrayList<>();
			for (Field f : clazz.getFields()) {
				fields.add(f.getName());
			}

			telemetry.addLine("--- Public Methods ---");
			for (int i = 0; i < Math.min(methods.size(), 20); i++) {
				telemetry.addLine(methods.get(i));
			}
			
			telemetry.addLine("--- Public Fields ---");
			for (int i = 0; i < Math.min(fields.size(), 20); i++) {
				telemetry.addLine(fields.get(i));
			}

			telemetry.update();
			sleep(1000);
		}
	}
}
