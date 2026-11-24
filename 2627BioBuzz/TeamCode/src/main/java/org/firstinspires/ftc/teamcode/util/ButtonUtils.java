package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * One-shot button detection for a gamepad.
 * <p>
 * {@code pressed(button)} returns {@code true} exactly ONCE per physical press:
 * <ul>
 *   <li>First call after the button goes down → {@code true}</li>
 *   <li>Further calls while it is still held → {@code false}</li>
 *   <li>After release and a new press → {@code true} again</li>
 * </ul>
 * <p>
 * Call {@link #update()} once at the start of every loop so releases are always
 * noticed, even if you did not ask about that button during the release.
 */
public class ButtonUtils {

    /**
     * Enum representing all detectable gamepad buttons.
     */
    public enum Button {
        /** Cross/A button (bottom face button) */
        A,
        /** Circle/B button (right face button) */
        B,
        /** Square/X button (left face button) */
        X,
        /** Triangle/Y button (top face button) */
        Y,
        /** D-pad up */
        DPAD_UP,
        /** D-pad down */
        DPAD_DOWN,
        /** D-pad left */
        DPAD_LEFT,
        /** D-pad right */
        DPAD_RIGHT,
        /** Left bumper (LB/L1) */
        LEFT_BUMPER,
        /** Right bumper (RB/R1) */
        RIGHT_BUMPER,
        /** Left trigger (LT/L2) - treated as button when past deadzone */
        LEFT_TRIGGER,
        /** Right trigger (RT/R2) - treated as button when past deadzone */
        RIGHT_TRIGGER,
        /** Left stick button (L3) */
        LEFT_STICK_BUTTON,
        /** Right stick button (R3) */
        RIGHT_STICK_BUTTON,
        /** Start/Options button */
        START,
        /** Back/Share button */
        BACK,
        /** Guide/Home button */
        GUIDE
    }

    private final Gamepad gamepad;

    // true = button is released/unused and the next press may fire
    private final boolean[] armed = new boolean[Button.values().length];

    /**
     * Creates a new ButtonUtils for the given gamepad.
     *
     * @param gamepad the gamepad to track (typically {@code gamepad1} or {@code gamepad2})
     */
    public ButtonUtils(Gamepad gamepad) {
        this.gamepad = gamepad;
        for (int i = 0; i < armed.length; i++) {
            armed[i] = true;
        }
    }

    /**
     * Call once per loop at the start of the loop.
     * <p>
     * Re-arms every button that is currently released, ensuring that the next
     * press will be detected even if {@link #pressed(Button)} wasn't called
     * for that button during the release frame.
     */
    public void update() {
        for (Button button : Button.values()) {
            if (!isDown(button)) {
                armed[button.ordinal()] = true;
            }
        }
    }

    /**
     * Returns {@code true} only once per press of the given button.
     * <p>
     * This is the primary method for detecting button presses for actions
     * that should trigger once (toggling modes, firing projectiles, etc.).
     *
     * @param button the button to check
     * @return {@code true} on the first loop where the button is detected as
     *         pressed after being released; {@code false} otherwise
     */
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

    /**
     * Returns the raw button state: {@code true} for as long as the button is held.
     * <p>
     * Use this for actions that should continue while the button is held
     * (e.g., driving, running an intake).
     *
     * @param button the button to check
     * @return {@code true} if the button is currently pressed
     */
    public boolean isDown(Button button) {
        switch (button) {
            case A:
                return gamepad.a;
            case B:
                return gamepad.b;
            case X:
                return gamepad.x;
            case Y:
                return gamepad.y;
            case DPAD_UP:
                return gamepad.dpad_up;
            case DPAD_DOWN:
                return gamepad.dpad_down;
            case DPAD_LEFT:
                return gamepad.dpad_left;
            case DPAD_RIGHT:
                return gamepad.dpad_right;
            case LEFT_BUMPER:
                return gamepad.left_bumper;
            case RIGHT_BUMPER:
                return gamepad.right_bumper;
            case LEFT_TRIGGER:
                return gamepad.left_trigger > RobotConstants.GAMEPAD_DEADZONE;
            case RIGHT_TRIGGER:
                return gamepad.right_trigger > RobotConstants.GAMEPAD_DEADZONE;
            case LEFT_STICK_BUTTON:
                return gamepad.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return gamepad.right_stick_button;
            case START:
                return gamepad.start;
            case BACK:
                return gamepad.back;
            case GUIDE:
                return gamepad.guide;
            default:
                return false;
        }
    }
}