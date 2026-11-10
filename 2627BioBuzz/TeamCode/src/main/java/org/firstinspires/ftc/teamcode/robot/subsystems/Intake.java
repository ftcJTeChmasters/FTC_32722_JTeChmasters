package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

import java.util.ArrayDeque;

/**
 * Intake subsystem.
 *
 * Owns the hardware for this mechanism and exposes high-level control
 * methods. Call update() once per loop if this subsystem needs
 * continuous control (e.g. a PID loop).
 */
public class Intake {
    private final DcMotor intakeMotor;
    private final MotorUtils motorUtils = new MotorUtils();
    private int currentState = RobotConstants.INTAKE_MOTOR_STOPED;

    // Speed history used for stability detection
    private static class Sample {
        final long time;
        final double speed;
        Sample(long time, double speed) {
            this.time = time;
            this.speed = speed;
        } 
    }

    private final ArrayDeque<Sample> history = new ArrayDeque<>();
    private long historyStartMs = -1;   // when we started collecting the current history
    private long stateStartMs;          // when the current state was entered
    public boolean enabled = true;

    // Stall recovery: 0 = none, 1 = reversing, 2 = forward super retry
    private int recoveryStage = 0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        stateStartMs = System.currentTimeMillis();
    }

    public double getSpeed() {
        return motorUtils.getMotorVelocityTicksPerSec(intakeMotor);
    }

    public void setSpeed(double speed) {
        intakeMotor.setPower(speed);
    }

    /** Call this once per loop from the OpMode, if needed. */
    public void update() {
        long now = System.currentTimeMillis();
        double speed = getSpeed();
        double absSpeed = Math.abs(speed);

        // Give the motor time to spin up / change direction before judging it
        boolean settled = now - stateStartMs >= RobotConstants.INTAKE_STATE_GRACE_MS;
        boolean stalled = settled && absSpeed < RobotConstants.INTAKE_MOTOR_STALL_SPEED;

        if (enabled) {
            if (currentState == RobotConstants.INTAKE_MOTOR_IDLE) {
                // Idle: hold idle speed, leave idle when the speed is disturbed
                setSpeed(RobotConstants.INTAKE_MOTOR_IDLE_SPEED);

                if (now - stateStartMs < RobotConstants.INTAKE_IDLE_SETTLE_MS) {
                    // Let the motor settle to idle speed first, don't record yet
                    clearHistory();
                } else {
                    recordSample(speed, now);
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
                } else if (now - stateStartMs >= RobotConstants.INTAKE_REVERSE_TIME_MS) {
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
                        enterState(RobotConstants.INTAKE_MOTOR_STOPED, now);
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
                    recordSample(speed, now);
                    if (isStable(now)) {
                        enterState(RobotConstants.INTAKE_MOTOR_IDLE, now);
                        setSpeed(RobotConstants.INTAKE_MOTOR_IDLE_SPEED);
                    }
                }

            } else {
                // STOPPED
                if (now - stateStartMs >= RobotConstants.INTAKE_STOP_TIME_MS) {
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
            currentState = RobotConstants.INTAKE_MOTOR_STOPED;
            setSpeed(0.0);
        }
    }

    // ---- State helper ----

    private void enterState(int state, long now) {
        currentState = state;
        stateStartMs = now;
        clearHistory();
    }

    // ---- Stability helpers ----

    private void recordSample(double speed, long now) {
        if (history.isEmpty() && historyStartMs < 0) {
            historyStartMs = now;
        }
        history.addLast(new Sample(now, speed));
        // Drop samples older than the window
        while (!history.isEmpty()
                && now - history.peekFirst().time > RobotConstants.INTAKE_STABLE_TIME_MS) {
            history.removeFirst();
        }
    }

    /** True if we have a full window of data and the average sample-to-sample change is small. */
    private boolean isStable(long now) {
        if (historyStartMs < 0 || history.size() < 2) return false;
        if (now - historyStartMs < RobotConstants.INTAKE_STABLE_TIME_MS) return false;

        double totalDiff = 0;
        Sample prev = null;
        for (Sample s : history) {
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
        for (Sample s : history) sum += s.speed;
        return sum / history.size();
    }

    private void clearHistory() {
        history.clear();
        historyStartMs = -1;
    }
}