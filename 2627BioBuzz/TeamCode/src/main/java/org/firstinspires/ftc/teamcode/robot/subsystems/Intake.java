package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;
import org.firstinspires.ftc.teamcode.util.EmergencyStop;
import org.firstinspires.ftc.teamcode.util.MotorErrorTracker;
import org.firstinspires.ftc.teamcode.util.MotorFaultDetector;
import org.firstinspires.ftc.teamcode.util.MotorUtils;

import java.util.ArrayDeque;

/**
 * Intake subsystem with automatic jam detection and recovery.
 */
public class Intake {
    private final DcMotor intakeMotor;
    private final MotorFaultDetector faultDetector;

    /** Current state of the intake state machine. */
    public int currentState = RobotConstants.INTAKE_MOTOR_STOPPED;

    private static class SpeedSample {
        final long time;
        final double speed;

        SpeedSample(long time, double speed) {
            this.time = time;
            this.speed = speed;
        }
    }

    private final ArrayDeque<SpeedSample> history = new ArrayDeque<>();
    private long historyStartMilliseconds = -1;
    private long stateStartMilliseconds;
    public boolean enabled = true;
    private boolean wasEnabled = true;

    // Stall recovery: 0 = none, 1 = reversing, 2 = forward super retry
    private int recoveryStage = 0;

    private double lastSpeed = 0.0;
    private double lastCommandedPower = 0.0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, HardwareNames.INTAKE_MOTOR);
        MotorUtils.clearMotorState(intakeMotor);

        faultDetector = new MotorFaultDetector(intakeMotor, "intake", "intake");
        faultDetector.setPresetThresholds("intake");
        MotorErrorTracker.getInstance().register(faultDetector);

        stateStartMilliseconds = System.currentTimeMillis();
    }

    /** Returns the current motor speed in encoder ticks per second. */
    public double getSpeed() {
        return MotorUtils.getMotorVelocityTicksPerSec(intakeMotor);
    }

    /** Returns the most recently sampled speed from {@link #update()}. */
    public double getLastSpeed() {
        return lastSpeed;
    }

    /** Returns the last commanded motor power (-1.0 to 1.0). */
    public double getMotorPower() {
        return lastCommandedPower;
    }

    /** Sets the motor power directly (bypasses state machine). */
    public void setSpeed(double speed) {
        lastCommandedPower = speed;
        intakeMotor.setPower(speed);
    }

    public static final String SUBSYSTEM_KEY = "intake";

    /** Enables or disables the intake cleanly, resetting state timers and history. */
    public void setEnabled(boolean enable) {
        if (enable && EmergencyStop.getInstance().isSubsystemStopped(SUBSYSTEM_KEY)) {
            return;
        }
        long now = System.currentTimeMillis();
        this.enabled = enable;
        this.wasEnabled = enable;
        this.recoveryStage = 0;
        if (enable) {
            enterState(RobotConstants.INTAKE_MOTOR_IDLE, now);
        } else {
            enterState(RobotConstants.INTAKE_MOTOR_STOPPED, now);
            setSpeed(0.0);
        }
    }

    /**
     * Toggles intake enabled/disabled cleanly.
     * If an Auto E-Stop is active on the intake, the first press clears the
     * E-Stop latch (keeping the intake off) so the driver can safely restart.
     */
    public void toggle() {
        EmergencyStop eStop = EmergencyStop.getInstance();
        if (eStop.isGlobalEStopped()) {
            return;
        }
        if (eStop.isSubsystemAutoEStopped(SUBSYSTEM_KEY)) {
            clearAutoEStop();
            return;
        }
        setEnabled(!enabled);
    }

    /** Clears an active Auto E-Stop on the intake and resets its fault detector. */
    public void clearAutoEStop() {
        EmergencyStop.getInstance().clearSubsystem(SUBSYSTEM_KEY);
        faultDetector.reset();
        enabled = false;
        wasEnabled = false;
        recoveryStage = 0;
        enterState(RobotConstants.INTAKE_MOTOR_STOPPED, System.currentTimeMillis());
        setSpeed(0.0);
    }

    /** Returns true if the intake is currently locked by global or auto E-Stop. */
    public boolean isEStopped() {
        return EmergencyStop.getInstance().isSubsystemStopped(SUBSYSTEM_KEY);
    }

    /** Triggers manual reverse jam-clearing if enabled (resets state timer properly). */
    public void triggerReverse() {
        if (EmergencyStop.getInstance().isSubsystemStopped(SUBSYSTEM_KEY)) {
            return;
        }
        if (!enabled) {
            setEnabled(true);
        }
        recoveryStage = 0;
        enterState(RobotConstants.INTAKE_MOTOR_REVERSE, System.currentTimeMillis());
        setSpeed(RobotConstants.INTAKE_MOTOR_REVERSE_SPEED);
    }

    public boolean isStallDetected() {
        return faultDetector.isStallDetected();
    }

    public boolean isEncoderFault() {
        return faultDetector.isEncoderFault();
    }

    public int getRecoveryStage() {
        return recoveryStage;
    }

    /** Returns human-readable name for the current intake state. */
    public String getStateName() {
        switch (currentState) {
            case RobotConstants.INTAKE_MOTOR_IDLE:
                return "IDLE";
            case RobotConstants.INTAKE_MOTOR_NORMAL:
                return "NORMAL";
            case RobotConstants.INTAKE_MOTOR_SUPER:
                return "SUPER";
            case RobotConstants.INTAKE_MOTOR_REVERSE:
                return "REVERSE";
            case RobotConstants.INTAKE_MOTOR_STOPPED:
            default:
                return "STOPPED";
        }
    }

    /**
     * Call this once per loop from the OpMode.
     */
    public void update() {
        long now = System.currentTimeMillis();
        double speed = getSpeed();
        lastSpeed = speed;
        double absSpeed = Math.abs(speed);

        EmergencyStop eStop = EmergencyStop.getInstance();
        if (eStop.isSubsystemStopped(SUBSYSTEM_KEY)) {
            enabled = false;
            wasEnabled = false;
            recoveryStage = 0;
            enterState(RobotConstants.INTAKE_MOTOR_STOPPED, now);
            setSpeed(0.0);
            faultDetector.update(0.0, intakeMotor.getCurrentPosition(), speed);
            reportTelemetry(speed);
            return;
        }

        long elapsedInState = now - stateStartMilliseconds;
        boolean settled = elapsedInState >= RobotConstants.INTAKE_STATE_GRACE_MS;
        boolean spinupSettled = elapsedInState >= RobotConstants.INTAKE_SPINUP_GRACE_MS;
        boolean stalled = settled && absSpeed < RobotConstants.INTAKE_MOTOR_STALL_SPEED;

        if (enabled) {
            if (!wasEnabled) {
                wasEnabled = true;
                recoveryStage = 0;
                enterState(RobotConstants.INTAKE_MOTOR_IDLE, now);
            }

            if (currentState == RobotConstants.INTAKE_MOTOR_IDLE) {
                setSpeed(RobotConstants.INTAKE_MOTOR_IDLE_SPEED);

                if (elapsedInState < RobotConstants.INTAKE_IDLE_SETTLE_MS) {
                    clearHistory();
                } else {
                    recordSpeedSample(speed, now);
                    if (Math.abs(speed - averageSpeed()) > RobotConstants.INTAKE_IDLE_EXIT_THRESHOLD) {
                        enterState(RobotConstants.INTAKE_MOTOR_NORMAL, now);
                        setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);
                    }
                }

            } else if (currentState == RobotConstants.INTAKE_MOTOR_REVERSE) {
                setSpeed(RobotConstants.INTAKE_MOTOR_REVERSE_SPEED);

                if (stalled) {
                    recoveryStage = 2;
                    enterState(RobotConstants.INTAKE_MOTOR_SUPER, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_SUPER_SPEED);
                } else if (elapsedInState >= RobotConstants.INTAKE_REVERSE_TIME_MS) {
                    recoveryStage = 0;
                    enterState(RobotConstants.INTAKE_MOTOR_NORMAL, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);
                }

            } else if (currentState == RobotConstants.INTAKE_MOTOR_SUPER) {
                if (stalled) {
                    if (recoveryStage >= 2) {
                        recoveryStage = 0;
                        enterState(RobotConstants.INTAKE_MOTOR_STOPPED, now);
                        setSpeed(0.0);
                        if (RobotConstants.AUTO_ESTOP_INTAKE_ENABLED
                                && RobotConstants.AUTO_ESTOP_INTAKE_ON_JAM_FAIL) {
                            enabled = false;
                            wasEnabled = false;
                            eStop.triggerSubsystem(SUBSYSTEM_KEY, "JAMMED: Reverse + super retry failed");
                        }
                    } else {
                        recoveryStage = 1;
                        enterState(RobotConstants.INTAKE_MOTOR_REVERSE, now);
                        setSpeed(RobotConstants.INTAKE_MOTOR_REVERSE_SPEED);
                    }
                } else if (spinupSettled && absSpeed >= RobotConstants.INTAKE_MOTOR_SUPER_START_SPEED) {
                    recoveryStage = 0;
                    enterState(RobotConstants.INTAKE_MOTOR_NORMAL, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);
                } else {
                    setSpeed(RobotConstants.INTAKE_MOTOR_SUPER_SPEED);
                }

            } else if (currentState == RobotConstants.INTAKE_MOTOR_NORMAL) {
                if (stalled) {
                    recoveryStage = 1;
                    enterState(RobotConstants.INTAKE_MOTOR_REVERSE, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_REVERSE_SPEED);
                } else if (spinupSettled && absSpeed < RobotConstants.INTAKE_MOTOR_SUPER_START_SPEED) {
                    enterState(RobotConstants.INTAKE_MOTOR_SUPER, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_SUPER_SPEED);
                } else {
                    setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);

                    recordSpeedSample(speed, now);
                    if (isStable(now)) {
                        enterState(RobotConstants.INTAKE_MOTOR_IDLE, now);
                        setSpeed(RobotConstants.INTAKE_MOTOR_IDLE_SPEED);
                    }
                }

            } else {
                // STOPPED
                if (elapsedInState >= RobotConstants.INTAKE_STOP_TIME_MS) {
                    enterState(RobotConstants.INTAKE_MOTOR_IDLE, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_IDLE_SPEED);
                } else if (elapsedInState >= RobotConstants.INTAKE_STOP_COAST_GRACE_MS
                        && absSpeed > RobotConstants.INTAKE_MOTOR_START_SPEED) {
                    enterState(RobotConstants.INTAKE_MOTOR_NORMAL, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);
                } else {
                    setSpeed(0.0);
                }
            }
        } else {
            if (wasEnabled || currentState != RobotConstants.INTAKE_MOTOR_STOPPED) {
                wasEnabled = false;
                recoveryStage = 0;
                enterState(RobotConstants.INTAKE_MOTOR_STOPPED, now);
            }
            setSpeed(0.0);
        }

        // Update fault detector using the power commanded this loop
        faultDetector.update(lastCommandedPower, intakeMotor.getCurrentPosition(), speed);

        if (shouldTriggerAutoEStop()) {
            String reason = faultDetector.getPrimaryError().getShortLabel()
                    + ": " + faultDetector.getErrorMessage();
            eStop.triggerSubsystem(SUBSYSTEM_KEY, reason);
            enabled = false;
            wasEnabled = false;
            recoveryStage = 0;
            enterState(RobotConstants.INTAKE_MOTOR_STOPPED, now);
            setSpeed(0.0);
        }

        reportTelemetry(speed);
    }

    private boolean shouldTriggerAutoEStop() {
        if (!RobotConstants.AUTO_ESTOP_INTAKE_ENABLED) {
            return false;
        }
        return (RobotConstants.AUTO_ESTOP_INTAKE_ON_ENCODER_FAULT && faultDetector.isEncoderFault())
                || (RobotConstants.AUTO_ESTOP_INTAKE_ON_OVERCURRENT && faultDetector.isOvercurrentDetected());
    }

    private void reportTelemetry(double speed) {
        TelemetryServer server = TelemetryServer.getInstance();
        server.setSubsystemTelemetry("intake", "state", currentState);
        server.setSubsystemTelemetry("intake", "stateName", getStateName());
        server.setSubsystemTelemetry("intake", "speed", speed);
        server.setSubsystemTelemetry("intake", "enabled", enabled);
        server.setSubsystemTelemetry("intake", "stallDetected", isStallDetected());
        server.setSubsystemTelemetry("intake", "encoderFault", isEncoderFault());
        server.setSubsystemTelemetry("intake", "recoveryStage", recoveryStage);
        server.setSubsystemTelemetry("intake", "error", faultDetector.getPrimaryError().name());
        server.setSubsystemTelemetry("intake", "eStopped", isEStopped());
    }

    private void enterState(int state, long now) {
        currentState = state;
        stateStartMilliseconds = now;
        clearHistory();
    }

    private void recordSpeedSample(double speed, long now) {
        if (history.isEmpty() && historyStartMilliseconds < 0) {
            historyStartMilliseconds = now;
        }
        history.addLast(new SpeedSample(now, speed));
        while (!history.isEmpty()
                && now - history.peekFirst().time > RobotConstants.INTAKE_STABLE_TIME_MS) {
            history.removeFirst();
        }
    }

    private boolean isStable(long now) {
        if (historyStartMilliseconds < 0 || history.size() < 2) return false;
        if (now - historyStartMilliseconds < RobotConstants.INTAKE_STABLE_TIME_MS) return false;

        double totalDiff = 0;
        SpeedSample prev = null;
        for (SpeedSample sample : history) {
            if (prev != null) {
                totalDiff += Math.abs(sample.speed - prev.speed);
            }
            prev = sample;
        }
        double averageDiff = totalDiff / (history.size() - 1);
        return averageDiff < RobotConstants.INTAKE_STABLE_THRESHOLD;
    }

    private double averageSpeed() {
        if (history.isEmpty()) return 0;
        double sum = 0;
        for (SpeedSample sample : history) {
            sum += sample.speed;
        }
        return sum / history.size();
    }

    private void clearHistory() {
        history.clear();
        historyStartMilliseconds = -1;
    }
}
