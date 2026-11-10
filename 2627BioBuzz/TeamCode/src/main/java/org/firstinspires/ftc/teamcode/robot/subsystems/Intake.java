package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;

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
    private long speedChangeStartMs = 0;       // when speed was changed

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
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
        TelemetryServer.getInstance().setSubsystemTelemetry("Intake", "motorSpeed", speed);
        TelemetryServer.getInstance().setSubsystemTelemetry("Intake", "state", currentState);


        if (currentState == RobotConstants.INTAKE_MOTOR_IDLE) {
            // Idle: hold idle speed, leave idle when the speed is disturbed
            setSpeed(RobotConstants.INTAKE_MOTOR_IDLE_SPEED);

            if (now - speedChangeStartMs < RobotConstants.INTAKE_SETTLE_MS) {
                // Let the motor settle to idle speed first, don't record yet
                clearHistory();
            } else {
                recordSample(speed, now);
                if (Math.abs(speed - averageSpeed()) > RobotConstants.INTAKE_IDLE_EXIT_THRESHOLD) {
                    currentState = RobotConstants.INTAKE_MOTOR_NORMAL;
                    clearHistory();
                    setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);
                }
            }
        } else if (currentState > RobotConstants.INTAKE_MOTOR_STOPED) {
            recordSample(speed, now);
            if (speed < RobotConstants.INTAKE_MOTOR_STALL_SPEED) {
                currentState = RobotConstants.INTAKE_MOTOR_STOPED;
                speedChangeStartMs = now;
                clearHistory();
                setSpeed(0.0);
            } else if (speed < RobotConstants.INTAKE_MOTOR_SUPER_START_SPEED) {
                currentState = RobotConstants.INTAKE_MOTOR_SUPER;
                clearHistory();
                setSpeed(RobotConstants.INTAKE_MOTOR_SUPER_SPEED);
            } else {
                setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);
                currentState = RobotConstants.INTAKE_MOTOR_NORMAL;

                // Speed stable for the last 5 seconds -> go to idle
                if (isStable(now)) {
                    currentState = RobotConstants.INTAKE_MOTOR_IDLE;
                    speedChangeStartMs = now;
                    clearHistory();
                    setSpeed(RobotConstants.INTAKE_MOTOR_IDLE_SPEED);
                }
            }
        } else {
            if (now - speedChangeStartMs < RobotConstants.INTAKE_SETTLE_MS) {
                // Let the motor settle to idle speed first, don't record yet
                clearHistory();
            } else if (speed > RobotConstants.INTAKE_MOTOR_START_SPEED) {
                currentState = RobotConstants.INTAKE_MOTOR_NORMAL;
                setSpeed(RobotConstants.INTAKE_MOTOR_SPEED);
            } else {
                setSpeed(0.0);
            }
        }
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