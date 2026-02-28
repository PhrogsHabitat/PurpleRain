package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;

public class Controls
{
	private final Gamepad gamepad;
	private final Map<String, ButtonState> buttonStates = new HashMap<>();

	public Controls (Gamepad gamepad)
	{
		this.gamepad = gamepad;
	}

	public void update ()
	{
		// Standard buttons
		updateButton("a", gamepad.a);
		updateButton("b", gamepad.b);
		updateButton("x", gamepad.x);
		updateButton("y", gamepad.y);
		updateButton("dpad_up", gamepad.dpad_up);
		updateButton("dpad_down", gamepad.dpad_down);
		updateButton("dpad_left", gamepad.dpad_left);
		updateButton("dpad_right", gamepad.dpad_right);
		updateButton("left_bumper", gamepad.left_bumper);
		updateButton("right_bumper", gamepad.right_bumper);
		updateButton("left_stick_button", gamepad.left_stick_button);
		updateButton("right_stick_button", gamepad.right_stick_button);
		updateButton("start", gamepad.start);
		updateButton("select", gamepad.back);

		// Triggers as buttons
		updateButton("left_trigger", gamepad.left_trigger > 0.5);
		updateButton("right_trigger", gamepad.right_trigger > 0.5);

		// Touchpad (simplified - treat as button)
		updateButton("touchpad", gamepad.touchpad);
	}

	private void updateButton (String name, boolean currentState)
	{
		ButtonState state = buttonStates.getOrDefault(name, new ButtonState());
		state.previousPressed = state.currentPressed;
		state.currentPressed = currentState;
		buttonStates.put(name, state);
	}

	public boolean isPressed (String button)
	{
		ButtonState state = buttonStates.get(button);
		return state != null && state.currentPressed;
	}

	public boolean justPressed (String button)
	{
		ButtonState state = buttonStates.get(button);
		return state != null && state.currentPressed && !state.previousPressed;
	}

	// Stick and trigger getters
	public double getLeftStickY ()
	{
		double value = -gamepad.left_stick_y;
		return Math.abs(value) < Constants.JOYSTICK_DEADZONE ? 0.0 : value;
	}

	public double getLeftStickX ()
	{
		double value = gamepad.left_stick_x;
		return Math.abs(value) < Constants.JOYSTICK_DEADZONE ? 0.0 : value;
	}

	public double getRightStickY ()
	{
		double value = -gamepad.right_stick_y;
		return Math.abs(value) < Constants.JOYSTICK_DEADZONE ? 0.0 : value;
	}

	public double getRightStickX ()
	{
		double value = gamepad.right_stick_x;
		return Math.abs(value) < Constants.JOYSTICK_DEADZONE ? 0.0 : value;
	}

	public double getLeftTrigger ()
	{
		return gamepad.left_trigger;
	}

	public double getRightTrigger ()
	{
		return gamepad.right_trigger;
	}

	// PlayStation specific features
	public void vibrate (int durationMs)
	{
		try
		{
			gamepad.rumble(durationMs);
		} catch (Exception e)
		{
			// Fallback if rumble not supported
		}
	}

	public void setTriggerFeedback (float resistance)
	{
		// This would require custom hardware support
		// Placeholder for future implementation
	}

	public float getGyroYaw ()
	{
		// Placeholder - would need gyro implementation
		return 0.0f;
	}

	private static class ButtonState
	{
		boolean currentPressed = false;
		boolean previousPressed = false;
	}
}
