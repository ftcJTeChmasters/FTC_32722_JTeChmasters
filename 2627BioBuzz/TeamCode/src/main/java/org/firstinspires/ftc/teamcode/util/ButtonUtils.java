package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * One-shot button detection for a gamepad.
 *
 * pressed(button) returns true ONCE per physical press:
 *  - first call after the button goes down  -> true
 *  - further calls while it is still held   -> false
 *  - after release and a new press          -> true again
 *
 * Call update() once at the start of every loop so releases are always noticed,
 * even if you did not ask about that button during the release.
 */
public class ButtonUtils {

    public enum Button {
        A, B, X, Y,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER,
        LEFT_TRIGGER, RIGHT_TRIGGER,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        START, BACK, GUIDE
    }

    private final Gamepad gamepad;

    // true = button is released/unused and the next press may fire
    private final boolean[] armed = new boolean[Button.values().length];

    public ButtonUtils(Gamepad gamepad) {
        this.gamepad = gamepad;
        for (int i = 0; i < armed.length; i++) {
            armed[i] = true;
        }
    }

    /** Call once per loop. Re-arms every button that is currently released. */
    public void update() {
        for (Button button : Button.values()) {
            if (!isDown(button)) {
                armed[button.ordinal()] = true;
            }
        }
    }

    /** Returns true only once per press of the given button. */
    public boolean pressed(Button button) {
        int i = button.ordinal();

        if (!isDown(button)) {
            armed[i] = true;
            return false;
        }

        if (armed[i]) {
            armed[i] = false;
            return true;
        }
        return false;
    }

    /** Raw state: true for as long as the button is held. */
    public boolean isDown(Button button) {
        switch (button) {
            case A:                 return gamepad.a;
            case B:                 return gamepad.b;
            case X:                 return gamepad.x;
            case Y:                 return gamepad.y;
            case DPAD_UP:           return gamepad.dpad_up;
            case DPAD_DOWN:         return gamepad.dpad_down;
            case DPAD_LEFT:         return gamepad.dpad_left;
            case DPAD_RIGHT:        return gamepad.dpad_right;
            case LEFT_BUMPER:       return gamepad.left_bumper;
            case RIGHT_BUMPER:      return gamepad.right_bumper;
            case LEFT_TRIGGER:      return gamepad.left_trigger > RobotConstants.GAMEPAD_DEADZONE;
            case RIGHT_TRIGGER:     return gamepad.right_trigger > RobotConstants.GAMEPAD_DEADZONE;
            case LEFT_STICK_BUTTON: return gamepad.left_stick_button;
            case RIGHT_STICK_BUTTON:return gamepad.right_stick_button;
            case START:             return gamepad.start;
            case BACK:              return gamepad.back;
            case GUIDE:             return gamepad.guide;
            default:                return false;
        }
    }
}