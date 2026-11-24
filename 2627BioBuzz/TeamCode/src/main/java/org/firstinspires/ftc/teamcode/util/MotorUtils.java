package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.HashMap;
import java.util.Map;

/**
 * Utility class for motor-related calculations.
 * <p>
 * Provides velocity estimation for motors that don't implement {@link DcMotorEx}.
 * Each motor's velocity state is tracked independently using a map keyed by
 * the motor's identity hash code (stable for the lifetime of the motor object).
 */
public class MotorUtils {

    /** Per-motor velocity tracking state. */
    private static class MotorVelocityData {
        int lastPosition = 0;
        long lastTimeNanoseconds = 0;
    }

    /** Map from motor identity hash to its velocity state. */
    private static final Map<Integer, MotorVelocityData> motorStates = new HashMap<>();

    /**
     * Gets the motor velocity in encoder ticks per second.
     * <p>
     * For {@link DcMotorEx} instances, uses the built-in velocity estimator.
     * For regular {@link DcMotor} instances, computes velocity from position
     * deltas using per-motor state tracking.
     *
     * @param motor the motor to query
     * @return velocity in ticks per second, or 0 if motor is null
     */
    public static float getMotorVelocityTicksPerSec(DcMotor motor) {
        if (motor == null) {
            return 0f;
        }

        if (motor instanceof DcMotorEx) {
            return (float) ((DcMotorEx) motor).getVelocity();
        }

        int motorKey = System.identityHashCode(motor);
        MotorVelocityData state = motorStates.computeIfAbsent(motorKey, k -> new MotorVelocityData());

        int currentPosition = motor.getCurrentPosition();
        long now = System.nanoTime();

        if (state.lastTimeNanoseconds == 0) {
            state.lastPosition = currentPosition;
            state.lastTimeNanoseconds = now;
            return 0f;
        }

        double dt = (now - state.lastTimeNanoseconds) / 1e9;
        if (dt <= 0) {
            return 0f;
        }

        double velocity = (currentPosition - state.lastPosition) / dt;

        state.lastPosition = currentPosition;
        state.lastTimeNanoseconds = now;

        return (float) velocity;
    }

    /**
     * Clears the velocity state for a specific motor.
     * <p>
     * Call this when a motor is re-initialized or when you want to reset
     * the velocity estimation baseline.
     *
     * @param motor the motor whose state to clear
     */
    public static void clearMotorState(DcMotor motor) {
        if (motor != null) {
            motorStates.remove(System.identityHashCode(motor));
        }
    }

    /**
     * Clears all tracked motor velocity states.
     * <p>
     * Useful when switching OpModes or during robot initialization.
     */
    public static void clearAllMotorStates() {
        motorStates.clear();
    }
}
