package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;

import java.util.ArrayDeque;

/**
 * Intake subsystem with automatic jam detection and recovery.
 * <p>
 * State machine states:
 * <ul>
 *   <li>{@link RobotConstants#INTAKE_MOTOR_STOPPED} - Motor stopped, waiting</li>
 *   <li>{@link RobotConstants#INTAKE_MOTOR_IDLE} - Holding low speed, waiting for game piece</li>
 *   <li>{@link RobotConstants#INTAKE_MOTOR_NORMAL} - Normal intake operation</li>
 *   <li>{@link RobotConstants#INTAKE_MOTOR_SUPER} - High speed for pushing through resistance</li>
 *   <li>{@link RobotConstants#INTAKE_MOTOR_REVERSE} - Reversing to clear jam</li>
 * </ul>
 * <p>
 * Call {@link #update()} once per loop from the OpMode.
 */
public class Intake {
    private final DcMotor intakeMotor;
    private final MotorUtils motorUtils = new MotorUtils();

    /** Current state of the intake state machine. */
    public int currentState = RobotConstants.INTAKE_MOTOR_STOPPED;

    // Speed history used for stability detection
    private static class SpeedSample {
        final long time;
        final double speed;
        SpeedSample(long time, double speed) {
            this.time = time;
            this.speed = speed;
        }
    }

    private final ArrayDeque<SpeedSample> history = new ArrayDeque<>();
    private long historyStartMilliseconds = -1;   // when we started collecting the current history
    private long stateStartMilliseconds;          // when the current state was entered
    public boolean enabled = true;

    // Stall recovery: 0 = none, 1 = reversing, 2 = forward super retry
    private int recoveryStage = 0;

    // Fault detection
    private long lastPositionChangeMilliseconds = 0;
    private int lastPosition = 0;
    private boolean stallDetected = false;
    private boolean encoderFault = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, HardwareNames.INTAKE_MOTOR);

        // Clear any stale velocity state from previous OpMode runs
        MotorUtils.clearMotorState(intakeMotor);

        stateStartMilliseconds = System.currentTimeMillis();
        lastPositionChangeMilliseconds = System.currentTimeMillis();
        lastPosition = intakeMotor.getCurrentPosition();
    }

    /**
     * Returns the current motor speed in encoder ticks per second.
     *
     * @return motor velocity (ticks/sec)
     */
    public double getSpeed() {
        return motorUtils.getMotorVelocityTicksPerSec(intakeMotor);
    }

    /**
     * Sets the motor power directly (bypasses state machine).
     *
     * @param speed motor power (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        intakeMotor.setPower(speed);
    }

    /**
     * Returns whether a motor stall has been detected.
     *
     * @return true if the motor appears to be stalled
     */
    public boolean isStallDetected() {
        return stallDetected;
    }

    /**
     * Returns whether an encoder fault has been detected.
     *
     * @return true if the encoder appears to be malfunctioning
     */
    public boolean isEncoderFault() {
        return encoderFault;
    }

    /**
     * Call this once per loop from the OpMode.
     * <p>
     * Runs the intake state machine with automatic jam detection and recovery.
     * Also performs fault detection for motor stall and encoder issues.
     */
    public void update() {
        long now = System.currentTimeMillis();
        double speed = getSpeed();
        double absSpeed = Math.abs(speed);
        int currentPosition = intakeMotor.getCurrentPosition();

        // Fault detection: check for encoder health
        checkEncoderFault(now, currentPosition);

        // Fault detection: check for motor stall (no position change despite power)
        checkMotorStall(now, currentPosition, speed);

        // Give the motor time to spin up / change direction before judging it
        boolean settled = now - stateStartMilliseconds >= RobotConstants.INTAKE_STATE_GRACE_MS;
        boolean stalled = settled && absSpeed < RobotConstants.INTAKE_MOTOR_STALL_SPEED;

        if (enabled) {
            if (currentState == RobotConstants.INTAKE_MOTOR_IDLE) {
                // Idle: hold idle speed, leave idle when the speed is disturbed
                setSpeed(RobotConstants.INTAKE_MOTOR_IDLE_SPEED);

                if (now - stateStartMilliseconds < RobotConstants.INTAKE_IDLE_SETTLE_MS) {
                    // Let the motor settle to idle speed first, don't record yet
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
                    // Reverse stalled too -> try forwards in super mode
                    recoveryStage = 2;
                    enterState(RobotConstants.INTAKE_MOTOR_SUPER, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_SUPER_SPEED);
                } else if (now - stateStartMilliseconds >= RobotConstants.INTAKE_REVERSE_TIME_MS) {
                    // Reversed freely, jam is cleared -> back to normal
                    recoveryStage = 0;
                    enterState(RobotConstants.INTAKE_MOTOR_NORMAL, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);
                }

            } else if (currentState == RobotConstants.INTAKE_MOTOR_SUPER) {
                if (stalled) {
                    if (recoveryStage >= 2) {
                        // Super retry after reverse stalled as well -> stop
                        recoveryStage = 0;
                        enterState(RobotConstants.INTAKE_MOTOR_STOPPED, now);
                        setSpeed(0.0);
                    } else {
                        // First stall -> reverse first
                        recoveryStage = 1;
                        enterState(RobotConstants.INTAKE_MOTOR_REVERSE, now);
                        setSpeed(RobotConstants.INTAKE_MOTOR_REVERSE_SPEED);
                    }
                } else if (settled && absSpeed >= RobotConstants.INTAKE_MOTOR_SUPER_START_SPEED) {
                    // Pushed through, back to normal
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
                } else if (settled && absSpeed < RobotConstants.INTAKE_MOTOR_SUPER_START_SPEED) {
                    enterState(RobotConstants.INTAKE_MOTOR_SUPER, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_SUPER_SPEED);
                } else {
                    setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);

                    // Speed stable for the last 5 seconds -> go to idle
                    recordSpeedSample(speed, now);
                    if (isStable(now)) {
                        enterState(RobotConstants.INTAKE_MOTOR_IDLE, now);
                        setSpeed(RobotConstants.INTAKE_MOTOR_IDLE_SPEED);
                    }
                }

            } else {
                // STOPPED
                if (now - stateStartMilliseconds >= RobotConstants.INTAKE_STOP_TIME_MS) {
                    // Stopped long enough -> back to idle
                    enterState(RobotConstants.INTAKE_MOTOR_IDLE, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_IDLE_SPEED);
                } else if (absSpeed > RobotConstants.INTAKE_MOTOR_START_SPEED) {
                    enterState(RobotConstants.INTAKE_MOTOR_NORMAL, now);
                    setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);
                } else {
                    setSpeed(0.0);
                }
            }
        } else {
            currentState = RobotConstants.INTAKE_MOTOR_STOPPED;
            setSpeed(0.0);
        }

        // Report telemetry
        reportTelemetry(now, speed);
    }

    /**
     * Checks for encoder faults (stuck encoder, unrealistic jumps).
     */
    private void checkEncoderFault(long nowMilliseconds, int currentPosition) {
        // Check for stuck encoder (position hasn't changed in a long time while powered)
        if (currentPosition != lastPosition) {
            lastPosition = currentPosition;
            lastPositionChangeMilliseconds = nowMilliseconds;
            encoderFault = false;
        } else {
            // Position unchanged - check if we're commanding power
            double motorPower = intakeMotor.getPower();
            if (Math.abs(motorPower) > RobotConstants.INTAKE_ENCODER_FAULT_POWER_THRESHOLD
                    && nowMilliseconds - lastPositionChangeMilliseconds
                            > RobotConstants.INTAKE_ENCODER_FAULT_TIMEOUT_MS) {
                encoderFault = true;
            }
        }
    }

    /**
     * Checks for motor stall (powered but not moving).
     */
    private void checkMotorStall(long nowMilliseconds, int currentPosition, double speed) {
        double motorPower = intakeMotor.getPower();
        if (Math.abs(motorPower) > RobotConstants.INTAKE_STALL_POWER_THRESHOLD
                && Math.abs(speed) < RobotConstants.INTAKE_STALL_SPEED_THRESHOLD) {
            if (nowMilliseconds - lastPositionChangeMilliseconds
                    > (long) (RobotConstants.MOTOR_STALL_TIMEOUT_SECONDS * 1000)) {
                stallDetected = true;
            }
        } else {
            stallDetected = false;
        }
    }

    /**
     * Reports telemetry to TelemetryServer.
     */
    private void reportTelemetry(long now, double speed) {
        TelemetryServer.getInstance().setSubsystemTelemetry("intake", "state", currentState);
        TelemetryServer.getInstance().setSubsystemTelemetry("intake", "speed", speed);
        TelemetryServer.getInstance().setSubsystemTelemetry("intake", "enabled", enabled);
        TelemetryServer.getInstance().setSubsystemTelemetry("intake", "stallDetected", stallDetected);
        TelemetryServer.getInstance().setSubsystemTelemetry("intake", "encoderFault", encoderFault);
        TelemetryServer.getInstance().setSubsystemTelemetry("intake", "recoveryStage", recoveryStage);
    }

    // ---- State helper ----

    private void enterState(int state, long now) {
        currentState = state;
        stateStartMilliseconds = now;
        clearHistory();
    }

    // ---- Stability helpers ----

    private void recordSpeedSample(double speed, long now) {
        if (history.isEmpty() && historyStartMilliseconds < 0) {
            historyStartMilliseconds = now;
        }
        history.addLast(new SpeedSample(now, speed));
        // Drop samples older than the window
        while (!history.isEmpty()
                && now - history.peekFirst().time > RobotConstants.INTAKE_STABLE_TIME_MS) {
            history.removeFirst();
        }
    }

    /**
     * Returns true if we have a full window of data and the average sample-to-sample change is small.
     */
    private boolean isStable(long now) {
        if (historyStartMilliseconds < 0 || history.size() < 2) return false;
        if (now - historyStartMilliseconds < RobotConstants.INTAKE_STABLE_TIME_MS) return false;

        double totalDiff = 0;
        SpeedSample prev = null;
        for (SpeedSample s : history) {
            if (prev != null) {
                totalDiff += Math.abs(s.speed - prev.speed);
            }
            prev = s;
        }
        double averageDiff = totalDiff / (history.size() - 1);
        return averageDiff < RobotConstants.INTAKE_STABLE_THRESHOLD;
    }

    private double averageSpeed() {
        if (history.isEmpty()) return 0;
        double sum = 0;
        for (SpeedSample s : history) sum += s.speed;
        return sum / history.size();
    }

    private void clearHistory() {
        history.clear();
        historyStartMilliseconds = -1;
    }
}