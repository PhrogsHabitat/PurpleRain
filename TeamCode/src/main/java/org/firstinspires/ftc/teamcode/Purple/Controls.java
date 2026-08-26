package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;

public class Controls
{
	private static final long TAP_MS = 250;
	private static final long HOLD_MS = 500;
	private static final double TAP_MOVE_LIMIT = 0.25;
	private static final double SWIPE_MIN = 0.55;

	private final Gamepad gamepad;
	private final Map<String, ButtonState> buttonStates = new HashMap<>();
	private boolean touching = false;
	private boolean wasTouching = false;
	private boolean justTouched = false;
	private boolean justReleasedTouch = false;
	private boolean tapped = false;
	private boolean holding = false;
	private boolean held = false;
	private boolean holdReported = false;
	private boolean secondTouching = false;
	private double touchX = 0.0;
	private double touchY = 0.0;
	private double secondTouchX = 0.0;
	private double secondTouchY = 0.0;
	private double startTouchX = 0.0;
	private double startTouchY = 0.0;
	private long touchStartTime = 0;
	private Swipe swipe = Swipe.NONE;

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

		updateTouchpad();

		// Touchpad
		updateButton("touchpad", gamepad.touchpad);
		updateButton("touchpad_finger_1", touching);
		updateButton("touchpad_finger_2", secondTouching);
	}

	private void updateButton (String name, boolean currentState)
	{
		ButtonState state = buttonStates.getOrDefault(name, new ButtonState());
		state.previousPressed = state.currentPressed;
		state.currentPressed = currentState;
		buttonStates.put(name, state);
	}

	private void updateTouchpad ()
	{
		wasTouching = touching;
		touching = gamepad.touchpad_finger_1;
		secondTouching = gamepad.touchpad_finger_2;
		secondTouchX = gamepad.touchpad_finger_2_x;
		secondTouchY = gamepad.touchpad_finger_2_y;
		justTouched = touching && !wasTouching;
		justReleasedTouch = !touching && wasTouching;
		tapped = false;
		held = false;
		swipe = Swipe.NONE;

		if (justTouched)
		{
			touchX = gamepad.touchpad_finger_1_x;
			touchY = gamepad.touchpad_finger_1_y;
			startTouchX = touchX;
			startTouchY = touchY;
			touchStartTime = System.currentTimeMillis();
			holdReported = false;
		}

		if (touching)
		{
			touchX = gamepad.touchpad_finger_1_x;
			touchY = gamepad.touchpad_finger_1_y;
			holding = System.currentTimeMillis() - touchStartTime >= HOLD_MS;
			if (holding && !holdReported)
			{
				held = true;
				holdReported = true;
			}
		}
		else
		{
			holding = false;
		}

		if (justReleasedTouch)
		{
			double deltaX = touchX - startTouchX;
			double deltaY = touchY - startTouchY;
			double distance = Math.hypot(deltaX, deltaY);
			long touchTime = System.currentTimeMillis() - touchStartTime;

			if (touchTime <= TAP_MS && distance < TAP_MOVE_LIMIT)
			{
				tapped = true;
			}

			if (distance >= SWIPE_MIN)
			{
				swipe = Math.abs(deltaX) > Math.abs(deltaY) ?
						(deltaX > 0 ? Swipe.RIGHT : Swipe.LEFT) :
						(deltaY > 0 ? Swipe.UP : Swipe.DOWN);
			}
		}
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

	public boolean isTouching ()
	{
		return touching;
	}

	public boolean isSecondTouching ()
	{
		return secondTouching;
	}

	public boolean justTouched ()
	{
		return justTouched;
	}

	public boolean justReleasedTouch ()
	{
		return justReleasedTouch;
	}

	public boolean justTapped ()
	{
		return tapped;
	}

	public boolean isHolding ()
	{
		return holding;
	}

	public boolean justHeld ()
	{
		return held;
	}

	public boolean justSwiped ()
	{
		return swipe != Swipe.NONE;
	}

	public Swipe getSwipe ()
	{
		return swipe;
	}

	public double getTouchX ()
	{
		return touchX;
	}

	public double getTouchY ()
	{
		return touchY;
	}

	public double getSecondTouchX ()
	{
		return secondTouchX;
	}

	public double getSecondTouchY ()
	{
		return secondTouchY;
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

	public enum Swipe
	{
		NONE,
		LEFT,
		RIGHT,
		UP,
		DOWN
	}
}
