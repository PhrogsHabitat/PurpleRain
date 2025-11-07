// File: Controls.java
// TODO: Change this to set custom keybinds, then use those keybinds as "actions" in the OpMode. (e.g. - Controls.ACCEPT)
package org.firstinspires.ftc.teamcode.Purple;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Purple.Constants;

import java.util.HashMap;
import java.util.Map;

public class Controls
{
    // Vision constants
    private final Gamepad gamepad;
    private final Map<String, ButtonState> buttonStates = new HashMap<>();

    public Controls(Gamepad gamepad)
    {
        this.gamepad = gamepad;
    }

    public void update()
    {
        // Update all button states
        updateButton("a", gamepad.a);
        updateButton("b", gamepad.b);
        updateButton("x", gamepad.x);
        updateButton("y", gamepad.y);
        updateButton("dpad_up", gamepad.dpad_up);
        updateButton("dpad_down", gamepad.dpad_down);
        updateButton("dpad_left", gamepad.dpad_left);
        updateButton("left_bumper", gamepad.left_bumper);
        updateButton("right_bumper", gamepad.right_bumper);


        // You can always add more buttons here!!!!
    }

    private void updateButton(String name, boolean currentState)
    {
        ButtonState state = buttonStates.getOrDefault(name, new ButtonState());
        state.previousPressed = state.currentPressed;
        state.currentPressed = currentState;
        buttonStates.put(name, state);
    }

    public boolean isPressed(String button)
    {
        ButtonState state = buttonStates.get(button);
        return state != null && state.currentPressed;
    }

    public boolean justPressed(String button)
    {
        ButtonState state = buttonStates.get(button);
        return state != null && state.currentPressed && !state.previousPressed;
    }

    public double getLeftStickY()
    {
        double value = -gamepad.left_stick_y; // Invert Y axis
        return Math.abs(value) < Constants.JOYSTICK_DEADZONE ? 0.0 : value;
    }

    public double getLeftStickX()
    {
        double value = gamepad.left_stick_x;
        return Math.abs(value) < Constants.JOYSTICK_DEADZONE ? 0.0 : value;
    }

    public double getRightStickY()
    {
        double value = -gamepad.right_stick_y; // Invert Y axis
        return Math.abs(value) < Constants.JOYSTICK_DEADZONE ? 0.0 : value;
    }

    public double getRightStickX()
    {
        double value = gamepad.right_stick_x;
        return Math.abs(value) < Constants.JOYSTICK_DEADZONE ? 0.0 : value;
    }

    public double getRightTrigger()
    {
        return gamepad.right_trigger;
    }

    public double getLeftTrigger()
    {
        return gamepad.left_trigger;
    }

    private static class ButtonState
    {
        boolean currentPressed = false;
        boolean previousPressed = false;
    }
}