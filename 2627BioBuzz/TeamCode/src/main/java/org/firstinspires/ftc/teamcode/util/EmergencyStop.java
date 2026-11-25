package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Centralized Emergency Stop (E-Stop) manager for both:
 * <ul>
 *   <li><b>Global manual E-Stop</b>: triggered by squeezing both gamepad triggers
 *       (&gt; {@link RobotConstants#ESTOP_TRIGGER_THRESHOLD}) simultaneously for
 *       {@link RobotConstants#ESTOP_TRIGGER_HOLD_MS} ms — hard to hit accidentally,
 *       fast and natural under stress. Cleared deliberately by holding {@code BACK}
 *       for {@link RobotConstants#ESTOP_RESET_HOLD_MS} ms.</li>
 *   <li><b>Per-subsystem Auto E-Stops</b>: isolates and shuts down an individual
 *       mechanism (e.g., jammed FlyWheel, jammed/faulted Intake, overcurrent motor)
 *       without disabling the rest of the robot.</li>
 * </ul>
 */
public final class EmergencyStop {

    private static final EmergencyStop INSTANCE = new EmergencyStop();

    private volatile boolean globalEStopped = false;
    private volatile String globalReason = "";
    private final Map<String, String> subsystemEStops = new ConcurrentHashMap<>();

    private long triggerHoldStartMs = -1;
    private long resetHoldStartMs = -1;
    private boolean requireTriggerRelease = false;
    private boolean requireResetRelease = false;

    private EmergencyStop() { }

    public static EmergencyStop getInstance() {
        return INSTANCE;
    }

    /**
     * Resets all global and subsystem E-Stop latches and hold timers.
     * Call on robot initialization so state never leaks across OpModes.
     */
    public void resetAll() {
        globalEStopped = false;
        globalReason = "";
        subsystemEStops.clear();
        triggerHoldStartMs = -1;
        resetHoldStartMs = -1;
        requireTriggerRelease = false;
        requireResetRelease = false;
    }

    /**
     * Updates global E-Stop trigger and reset detection from raw gamepad values.
     * Supports both primary and secondary drivers.
     */
    public void updateGamepadInputs(
            double leftTrigger1,
            double rightTrigger1,
            boolean backButton1,
            double leftTrigger2,
            double rightTrigger2,
            boolean backButton2
    ) {
        long now = System.currentTimeMillis();

        boolean panicTriggersHeld =
                (leftTrigger1 > RobotConstants.ESTOP_TRIGGER_THRESHOLD
                        && rightTrigger1 > RobotConstants.ESTOP_TRIGGER_THRESHOLD)
                || (leftTrigger2 > RobotConstants.ESTOP_TRIGGER_THRESHOLD
                        && rightTrigger2 > RobotConstants.ESTOP_TRIGGER_THRESHOLD);

        boolean resetHeld = backButton1 || backButton2;

        if (!panicTriggersHeld) {
            triggerHoldStartMs = -1;
            requireTriggerRelease = false;
        } else if (!requireTriggerRelease && !globalEStopped) {
            if (triggerHoldStartMs < 0) {
                triggerHoldStartMs = now;
            }
            if ((now - triggerHoldStartMs) >= RobotConstants.ESTOP_TRIGGER_HOLD_MS) {
                triggerGlobal("Manual E-Stop (Triggers)");
                requireTriggerRelease = true;
            }
        }

        if (!resetHeld) {
            resetHoldStartMs = -1;
            requireResetRelease = false;
        } else if (!requireResetRelease && hasAnyStop()) {
            if (resetHoldStartMs < 0) {
                resetHoldStartMs = now;
            }
            if ((now - resetHoldStartMs) >= RobotConstants.ESTOP_RESET_HOLD_MS) {
                clearGlobal();
                clearAllSubsystems();
                requireResetRelease = true;
                if (panicTriggersHeld) {
                    requireTriggerRelease = true;
                }
            }
        }
    }

    /** Triggers a global latched E-Stop with the given reason. */
    public void triggerGlobal(String reason) {
        this.globalEStopped = true;
        this.globalReason = (reason == null || reason.isEmpty()) ? "Manual E-Stop" : reason;
    }

    /** Clears the global E-Stop latch. */
    public void clearGlobal() {
        this.globalEStopped = false;
        this.globalReason = "";
    }

    /** Returns true if the global E-Stop latch is active. */
    public boolean isGlobalEStopped() {
        return globalEStopped;
    }

    /** Returns the reason for the active global E-Stop, or empty string if inactive. */
    public String getGlobalReason() {
        return globalReason;
    }

    /** Triggers an Auto E-Stop latch on a single subsystem. */
    public void triggerSubsystem(String subsystem, String reason) {
        if (subsystem == null || subsystem.isEmpty()) return;
        String safeReason = (reason == null || reason.isEmpty()) ? "Fault detected" : reason;
        subsystemEStops.putIfAbsent(subsystem, safeReason);
    }

    /** Clears an Auto E-Stop latch for a single subsystem. */
    public void clearSubsystem(String subsystem) {
        if (subsystem != null) {
            subsystemEStops.remove(subsystem);
        }
    }

    /** Clears all subsystem Auto E-Stops (does not clear global E-Stop). */
    public void clearAllSubsystems() {
        subsystemEStops.clear();
    }

    /** Returns true if this subsystem is stopped by either global E-Stop or its own Auto E-Stop. */
    public boolean isSubsystemStopped(String subsystem) {
        if (globalEStopped) return true;
        return subsystem != null && subsystemEStops.containsKey(subsystem);
    }

    /** Returns true if this specific subsystem has an Auto E-Stop active. */
    public boolean isSubsystemAutoEStopped(String subsystem) {
        return subsystem != null && subsystemEStops.containsKey(subsystem);
    }

    /** Returns the Auto E-Stop reason for a subsystem, or empty string if none. */
    public String getSubsystemReason(String subsystem) {
        if (subsystem == null) return "";
        String reason = subsystemEStops.get(subsystem);
        return reason == null ? "" : reason;
    }

    /** Returns a snapshot map of all active subsystem Auto E-Stops (subsystem -> reason). */
    public Map<String, String> getActiveSubsystemEStops() {
        return Collections.unmodifiableMap(new LinkedHashMap<>(subsystemEStops));
    }

    /** Returns true if either a global E-Stop or any subsystem Auto E-Stop is active. */
    public boolean hasAnyStop() {
        return globalEStopped || !subsystemEStops.isEmpty();
    }

    /** Returns 0.0 to 1.0 progress while BACK is being held to clear an active E-Stop. */
    public double getResetHoldProgress() {
        if (resetHoldStartMs < 0 || !hasAnyStop() || requireResetRelease) {
            return 0.0;
        }
        long elapsed = System.currentTimeMillis() - resetHoldStartMs;
        return Math.max(0.0, Math.min(1.0, (double) elapsed / RobotConstants.ESTOP_RESET_HOLD_MS));
    }
}
