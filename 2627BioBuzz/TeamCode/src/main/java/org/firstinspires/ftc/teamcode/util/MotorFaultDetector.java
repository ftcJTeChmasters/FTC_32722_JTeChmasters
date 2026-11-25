package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Centralized motor fault detection utility.
 * <p>
 * Monitors a single {@link DcMotor} (or {@link DcMotorEx}) for:
 * <ul>
 *   <li>Missing hardware ({@link MotorError.ErrorType#HARDWARE_MISSING})</li>
 *   <li>Encoder faults ({@link MotorError.ErrorType#ENCODER_FAULT})</li>
 *   <li>Motor stalls ({@link MotorError.ErrorType#STALLED})</li>
 *   <li>Overcurrent spikes ({@link MotorError.ErrorType#OVERCURRENT}, DcMotorEx only)</li>
 *   <li>Reversed encoder direction ({@link MotorError.ErrorType#REVERSED_ENCODER})</li>
 *   <li>PID saturation &amp; speed tracking error (via {@link #updatePidStatus})</li>
 * </ul>
 * <p>
 * Automatically tracks fault timers only while power is actively applied above
 * threshold (preventing false faults immediately after sitting idle).
 */
public class MotorFaultDetector implements MotorError.FaultSource {

    private final DcMotor motor;
    private final String subsystem;
    private final String name;

    // Feature toggles (disable encoder checks for encoderless drive motors)
    private boolean encoderCheckEnabled = true;

    // Encoder fault thresholds
    private double encoderFaultPowerThreshold = RobotConstants.DRIVETRAIN_ENCODER_FAULT_POWER_THRESHOLD;
    private long encoderFaultTimeoutMs = RobotConstants.DRIVETRAIN_ENCODER_FAULT_TIMEOUT_MS;

    // Stall detection thresholds
    private double stallPowerThreshold = RobotConstants.DRIVETRAIN_STALL_POWER_THRESHOLD;
    private double stallSpeedThreshold = RobotConstants.DRIVETRAIN_STALL_SPEED_THRESHOLD;
    private long stallTimeoutMs = (long) (RobotConstants.MOTOR_STALL_TIMEOUT_SECONDS * 1000);

    // Overcurrent thresholds (DcMotorEx)
    private double overcurrentAmps = RobotConstants.MOTOR_OVERCURRENT_AMPS;
    private long overcurrentTimeoutMs = RobotConstants.MOTOR_OVERCURRENT_TIMEOUT_MS;

    // Reversed encoder thresholds
    private double reversedPowerThreshold = RobotConstants.MOTOR_REVERSED_ENCODER_POWER_THRESHOLD;
    private double reversedSpeedThreshold = RobotConstants.MOTOR_REVERSED_ENCODER_SPEED_THRESHOLD;
    private long reversedTimeoutMs = RobotConstants.MOTOR_REVERSED_ENCODER_TIMEOUT_MS;

    // PID fault thresholds
    private long pidSaturationTimeoutMs = RobotConstants.MOTOR_PID_SATURATION_TIMEOUT_MS;
    private long speedErrorTimeoutMs = RobotConstants.MOTOR_SPEED_ERROR_TIMEOUT_MS;

    // Internal timing & position state
    private int lastPosition = 0;
    private long lastPositionChangeMs = 0;
    private long encoderFaultTimerStartMs = -1;
    private long stallTimerStartMs = -1;
    private long overcurrentTimerStartMs = -1;
    private long reversedTimerStartMs = -1;
    private long pidSaturationTimerStartMs = -1;
    private long speedErrorTimerStartMs = -1;

    // Active fault flags
    private boolean hardwareMissing = false;
    private boolean encoderFault = false;
    private boolean stallDetected = false;
    private boolean overcurrentDetected = false;
    private boolean reversedEncoderDetected = false;
    private boolean pidSaturatedDetected = false;
    private boolean speedErrorDetected = false;

    // Last sampled metrics & diagnostics
    private double lastPower = 0.0;
    private double lastSpeed = 0.0;
    private double lastCurrentAmps = 0.0;
    private double lastTargetSpeed = 0.0;
    private int totalFaultCount = 0;
    private boolean hadActiveFault = false;
    private long lastUpdateMs = 0;

    /**
     * Creates a new MotorFaultDetector for the given motor and subsystem.
     *
     * @param motor     the motor to monitor (may be null to flag HARDWARE_MISSING)
     * @param subsystem subsystem name (e.g., "driveTrain", "intake", "flyWheel")
     * @param name      descriptive motor name (e.g., "leftDrive", "frontLeft")
     */
    public MotorFaultDetector(DcMotor motor, String subsystem, String name) {
        this.motor = motor;
        this.subsystem = subsystem == null ? "robot" : subsystem;
        this.name = name == null ? "motor" : name;
        this.lastUpdateMs = System.currentTimeMillis();
        if (motor == null) {
            this.hardwareMissing = true;
            this.totalFaultCount = 1;
            this.hadActiveFault = true;
        } else {
            this.lastPosition = motor.getCurrentPosition();
            this.lastPositionChangeMs = this.lastUpdateMs;
        }
    }

    /**
     * Creates a new MotorFaultDetector for the given motor.
     *
     * @param motor the motor to monitor
     * @param name  a descriptive name for telemetry (e.g., "leftDrive", "intake", "flywheel")
     */
    public MotorFaultDetector(DcMotor motor, String name) {
        this(motor, name, name);
    }

    /**
     * Creates a new MotorFaultDetector with default thresholds from RobotConstants.
     *
     * @param motor the motor to monitor
     */
    public MotorFaultDetector(DcMotor motor) {
        this(motor, "motor", "motor");
    }

    // ---- Configuration ----

    /**
     * Enables or disables encoder-dependent checks (encoder fault, stall, reversed encoder).
     * Useful when a drive motor runs without its encoder cable connected.
     */
    public void setEncoderCheckEnabled(boolean enabled) {
        this.encoderCheckEnabled = enabled;
        if (!enabled) {
            encoderFault = false;
            stallDetected = false;
            reversedEncoderDetected = false;
            encoderFaultTimerStartMs = -1;
            stallTimerStartMs = -1;
            reversedTimerStartMs = -1;
        }
    }

    public boolean isEncoderCheckEnabled() {
        return encoderCheckEnabled;
    }

    /**
     * Sets the encoder fault detection thresholds.
     *
     * @param powerThreshold minimum motor power (0.0-1.0) above which to check for encoder faults
     * @param timeoutMs      milliseconds with no position change while powered before declaring fault
     */
    public void setEncoderFaultThresholds(double powerThreshold, long timeoutMs) {
        this.encoderFaultPowerThreshold = Math.max(0, Math.min(1.0, powerThreshold));
        this.encoderFaultTimeoutMs = Math.max(0, timeoutMs);
    }

    /**
     * Sets the motor stall detection thresholds.
     *
     * @param powerThreshold minimum motor power (0.0-1.0) above which to check for stalls
     * @param speedThreshold speed (ticks/sec) below which motor is considered stalled
     * @param timeoutMs      milliseconds while powered and below speed before declaring stall
     */
    public void setStallThresholds(double powerThreshold, double speedThreshold, long timeoutMs) {
        this.stallPowerThreshold = Math.max(0, Math.min(1.0, powerThreshold));
        this.stallSpeedThreshold = Math.max(0, speedThreshold);
        this.stallTimeoutMs = Math.max(0, timeoutMs);
    }

    /**
     * Sets overcurrent detection thresholds (applies to {@link DcMotorEx} motors).
     */
    public void setOvercurrentThresholds(double amps, long timeoutMs) {
        this.overcurrentAmps = Math.max(0.5, amps);
        this.overcurrentTimeoutMs = Math.max(0, timeoutMs);
    }

    /**
     * Sets all thresholds at once using RobotConstants presets for a specific subsystem.
     *
     * @param subsystem one of "drivetrain", "intake", "flywheel"
     */
    public void setPresetThresholds(String subsystem) {
        switch (subsystem.toLowerCase(Locale.ROOT)) {
            case "drivetrain":
                setEncoderFaultThresholds(
                        RobotConstants.DRIVETRAIN_ENCODER_FAULT_POWER_THRESHOLD,
                        RobotConstants.DRIVETRAIN_ENCODER_FAULT_TIMEOUT_MS
                );
                setStallThresholds(
                        RobotConstants.DRIVETRAIN_STALL_POWER_THRESHOLD,
                        RobotConstants.DRIVETRAIN_STALL_SPEED_THRESHOLD,
                        (long) (RobotConstants.MOTOR_STALL_TIMEOUT_SECONDS * 1000)
                );
                break;
            case "intake":
                setEncoderFaultThresholds(
                        RobotConstants.INTAKE_ENCODER_FAULT_POWER_THRESHOLD,
                        RobotConstants.INTAKE_ENCODER_FAULT_TIMEOUT_MS
                );
                setStallThresholds(
                        RobotConstants.INTAKE_STALL_POWER_THRESHOLD,
                        RobotConstants.INTAKE_STALL_SPEED_THRESHOLD,
                        (long) (RobotConstants.MOTOR_STALL_TIMEOUT_SECONDS * 1000)
                );
                break;
            case "flywheel":
                setEncoderFaultThresholds(
                        RobotConstants.FLYWHEEL_ENCODER_FAULT_POWER_THRESHOLD,
                        RobotConstants.FLYWHEEL_ENCODER_FAULT_TIMEOUT_MS
                );
                setStallThresholds(
                        RobotConstants.FLYWHEEL_STALL_POWER_THRESHOLD,
                        RobotConstants.FLYWHEEL_STALL_SPEED_THRESHOLD,
                        (long) (RobotConstants.MOTOR_STALL_TIMEOUT_SECONDS * 1000)
                );
                break;
            default:
                setPresetThresholds("drivetrain");
        }
    }

    // ---- Main update loop ----

    /**
     * Updates fault detection state. Call once per control loop.
     *
     * @param motorPower      current motor power (-1.0 to 1.0)
     * @param currentPosition current encoder position
     * @param currentSpeed    current motor velocity in ticks/second
     */
    public void update(double motorPower, int currentPosition, double currentSpeed) {
        long now = System.currentTimeMillis();
        lastUpdateMs = now;
        lastPower = motorPower;
        lastSpeed = currentSpeed;

        if (motor == null) {
            hardwareMissing = true;
            updateFaultCount();
            return;
        }
        hardwareMissing = false;

        double absPower = Math.abs(motorPower);
        double absSpeed = Math.abs(currentSpeed);

        if (encoderCheckEnabled) {
            boolean positionChanged = currentPosition != lastPosition;
            if (positionChanged) {
                lastPosition = currentPosition;
                lastPositionChangeMs = now;
                encoderFault = false;
                encoderFaultTimerStartMs = -1;
            } else if (absPower > encoderFaultPowerThreshold) {
                if (encoderFaultTimerStartMs < 0) {
                    encoderFaultTimerStartMs = now;
                }
                if ((now - encoderFaultTimerStartMs) >= encoderFaultTimeoutMs) {
                    encoderFault = true;
                }
            } else {
                // Not meaningfully powered -> reset timer and clear transient fault
                encoderFaultTimerStartMs = -1;
                encoderFault = false;
            }

            // Stall detection: only when powered above threshold and moving too slowly.
            // If encoderFault is already active (literally zero ticks ever changing), keep
            // encoderFault as the distinct root cause unless speed is genuinely non-zero/oscillating.
            if (absPower > stallPowerThreshold && absSpeed < stallSpeedThreshold) {
                if (stallTimerStartMs < 0) {
                    stallTimerStartMs = now;
                }
                if ((now - stallTimerStartMs) >= stallTimeoutMs && !encoderFault) {
                    stallDetected = true;
                }
            } else {
                stallTimerStartMs = -1;
                stallDetected = false;
            }

            // Reversed encoder polarity detection
            if (absPower > reversedPowerThreshold
                    && absSpeed > reversedSpeedThreshold
                    && Math.signum(currentSpeed) != Math.signum(motorPower)) {
                if (reversedTimerStartMs < 0) {
                    reversedTimerStartMs = now;
                }
                if ((now - reversedTimerStartMs) >= reversedTimeoutMs) {
                    reversedEncoderDetected = true;
                }
            } else {
                reversedTimerStartMs = -1;
                reversedEncoderDetected = false;
            }
        } else {
            encoderFault = false;
            stallDetected = false;
            reversedEncoderDetected = false;
            encoderFaultTimerStartMs = -1;
            stallTimerStartMs = -1;
            reversedTimerStartMs = -1;
        }

        // Overcurrent detection for DcMotorEx
        if (motor instanceof DcMotorEx) {
            double amps = ((DcMotorEx) motor).getCurrent(CurrentUnit.AMPS);
            if (!Double.isNaN(amps) && !Double.isInfinite(amps)) {
                lastCurrentAmps = amps;
                if (amps >= overcurrentAmps) {
                    if (overcurrentTimerStartMs < 0) {
                        overcurrentTimerStartMs = now;
                    }
                    if ((now - overcurrentTimerStartMs) >= overcurrentTimeoutMs) {
                        overcurrentDetected = true;
                    }
                } else {
                    overcurrentTimerStartMs = -1;
                    overcurrentDetected = false;
                }
            }
        }

        updateFaultCount();
    }

    /**
     * Updates fault detection using motor's getPower() and getCurrentPosition().
     * Convenience overload that reads directly from the motor.
     *
     * @param currentSpeed current motor velocity in ticks/second
     */
    public void update(double currentSpeed) {
        if (motor == null) {
            update(0.0, 0, currentSpeed);
            return;
        }
        update(motor.getPower(), motor.getCurrentPosition(), currentSpeed);
    }

    /**
     * Updates optional PID velocity/position tracking fault states.
     *
     * @param active            whether closed-loop PID control is currently active
     * @param saturated         whether PID output hit output limits
     * @param targetSpeed       target velocity/setpoint
     * @param actualSpeed       actual measured velocity/value
     * @param errorThresholdAbs absolute error tolerance above which SPEED_ERROR is timed
     */
    public void updatePidStatus(
            boolean active,
            boolean saturated,
            double targetSpeed,
            double actualSpeed,
            double errorThresholdAbs
    ) {
        long now = System.currentTimeMillis();
        lastTargetSpeed = targetSpeed;

        if (!active) {
            pidSaturatedDetected = false;
            speedErrorDetected = false;
            pidSaturationTimerStartMs = -1;
            speedErrorTimerStartMs = -1;
            updateFaultCount();
            return;
        }

        if (saturated) {
            if (pidSaturationTimerStartMs < 0) {
                pidSaturationTimerStartMs = now;
            }
            if ((now - pidSaturationTimerStartMs) >= pidSaturationTimeoutMs) {
                pidSaturatedDetected = true;
            }
        } else {
            pidSaturationTimerStartMs = -1;
            pidSaturatedDetected = false;
        }

        double absError = Math.abs(targetSpeed - actualSpeed);
        if (absError > errorThresholdAbs) {
            if (speedErrorTimerStartMs < 0) {
                speedErrorTimerStartMs = now;
            }
            if ((now - speedErrorTimerStartMs) >= speedErrorTimeoutMs) {
                speedErrorDetected = true;
            }
        } else {
            speedErrorTimerStartMs = -1;
            speedErrorDetected = false;
        }

        updateFaultCount();
    }

    private void updateFaultCount() {
        boolean activeNow = hasAnyFault();
        if (activeNow && !hadActiveFault) {
            totalFaultCount++;
        }
        hadActiveFault = activeNow;
    }

    // ---- Status getters ----

    /** Returns true if the motor reference is null / missing. */
    public boolean isHardwareMissing() {
        return hardwareMissing;
    }

    /** Returns true if an encoder fault has been detected. */
    public boolean isEncoderFault() {
        return encoderFault;
    }

    /** Returns true if a motor stall has been detected. */
    public boolean isStallDetected() {
        return stallDetected;
    }

    /** Returns true if an overcurrent spike has been detected (DcMotorEx). */
    public boolean isOvercurrentDetected() {
        return overcurrentDetected;
    }

    /** Returns true if the encoder appears reversed relative to motor power. */
    public boolean isReversedEncoderDetected() {
        return reversedEncoderDetected;
    }

    /** Returns true if PID output has been saturated beyond timeout. */
    public boolean isPidSaturatedDetected() {
        return pidSaturatedDetected;
    }

    /** Returns true if speed tracking error exceeded tolerance beyond timeout. */
    public boolean isSpeedErrorDetected() {
        return speedErrorDetected;
    }

    /** Returns true if any fault or warning is currently active. */
    public boolean hasAnyFault() {
        return hardwareMissing
                || encoderFault
                || stallDetected
                || overcurrentDetected
                || reversedEncoderDetected
                || pidSaturatedDetected
                || speedErrorDetected;
    }

    /** Returns the highest-priority active error type. */
    public MotorError.ErrorType getPrimaryError() {
        if (hardwareMissing) return MotorError.ErrorType.HARDWARE_MISSING;
        if (overcurrentDetected) return MotorError.ErrorType.OVERCURRENT;
        if (encoderFault) return MotorError.ErrorType.ENCODER_FAULT;
        if (stallDetected) return MotorError.ErrorType.STALLED;
        if (reversedEncoderDetected) return MotorError.ErrorType.REVERSED_ENCODER;
        if (pidSaturatedDetected) return MotorError.ErrorType.PID_SATURATED;
        if (speedErrorDetected) return MotorError.ErrorType.SPEED_ERROR;
        return MotorError.ErrorType.NONE;
    }

    /** Returns the set of all currently active error types. */
    public EnumSet<MotorError.ErrorType> getActiveErrors() {
        EnumSet<MotorError.ErrorType> errors = EnumSet.noneOf(MotorError.ErrorType.class);
        if (hardwareMissing) errors.add(MotorError.ErrorType.HARDWARE_MISSING);
        if (encoderFault) errors.add(MotorError.ErrorType.ENCODER_FAULT);
        if (stallDetected) errors.add(MotorError.ErrorType.STALLED);
        if (overcurrentDetected) errors.add(MotorError.ErrorType.OVERCURRENT);
        if (reversedEncoderDetected) errors.add(MotorError.ErrorType.REVERSED_ENCODER);
        if (pidSaturatedDetected) errors.add(MotorError.ErrorType.PID_SATURATED);
        if (speedErrorDetected) errors.add(MotorError.ErrorType.SPEED_ERROR);
        return errors;
    }

    /** Returns a human-readable diagnostic message describing the current fault. */
    public String getErrorMessage() {
        MotorError.ErrorType primary = getPrimaryError();
        switch (primary) {
            case HARDWARE_MISSING:
                return "Motor not found in HardwareMap";
            case OVERCURRENT:
                return String.format(Locale.US, "High current %.1fA (pwr=%.2f)", lastCurrentAmps, lastPower);
            case ENCODER_FAULT:
                return String.format(Locale.US, "No encoder ticks at pwr=%.2f (check cable)", lastPower);
            case STALLED:
                return String.format(Locale.US, "Stalled at pwr=%.2f vel=%.0f t/s", lastPower, lastSpeed);
            case REVERSED_ENCODER:
                return String.format(Locale.US, "Encoder opposite power (pwr=%.2f vel=%.0f)", lastPower, lastSpeed);
            case PID_SATURATED:
                return String.format(Locale.US, "PID saturated (tgt=%.0f act=%.0f)", lastTargetSpeed, lastSpeed);
            case SPEED_ERROR:
                return String.format(Locale.US, "Speed off target (tgt=%.0f act=%.0f)", lastTargetSpeed, lastSpeed);
            case NONE:
            default:
                return "OK";
        }
    }

    @Override
    public MotorError.MotorErrorReport getReport() {
        EnumSet<MotorError.ErrorType> active = getActiveErrors();
        MotorError.ErrorType primary = getPrimaryError();
        MotorError.Severity severity = MotorError.Severity.OK;
        for (MotorError.ErrorType err : active) {
            if (err.getSeverity().isWorseThan(severity)) {
                severity = err.getSeverity();
            }
        }
        return new MotorError.MotorErrorReport(
                subsystem,
                name,
                primary,
                active,
                severity,
                getErrorMessage(),
                lastPower,
                lastSpeed,
                lastCurrentAmps,
                totalFaultCount,
                lastUpdateMs
        );
    }

    /** Returns the descriptive name of this detector. */
    public String getName() {
        return name;
    }

    /** Returns the subsystem name of this detector. */
    public String getSubsystem() {
        return subsystem;
    }

    /** Resets the fault detection state (e.g., after motor reconfiguration). */
    public void reset() {
        long now = System.currentTimeMillis();
        lastUpdateMs = now;
        if (motor != null) {
            lastPosition = motor.getCurrentPosition();
            lastPositionChangeMs = now;
            hardwareMissing = false;
        } else {
            hardwareMissing = true;
        }
        encoderFaultTimerStartMs = -1;
        stallTimerStartMs = -1;
        overcurrentTimerStartMs = -1;
        reversedTimerStartMs = -1;
        pidSaturationTimerStartMs = -1;
        speedErrorTimerStartMs = -1;

        encoderFault = false;
        stallDetected = false;
        overcurrentDetected = false;
        reversedEncoderDetected = false;
        pidSaturatedDetected = false;
        speedErrorDetected = false;
        hadActiveFault = hardwareMissing;
    }

    /**
     * Gets a snapshot of all fault states for telemetry.
     *
     * @return a map with fault status values
     */
    public Map<String, Object> getTelemetrySnapshot() {
        Map<String, Object> snapshot = new HashMap<>();
        snapshot.put("primaryError", getPrimaryError().name());
        snapshot.put("severity", getReport().getSeverity().name());
        snapshot.put("message", getErrorMessage());
        snapshot.put("encoderFault", encoderFault);
        snapshot.put("stallDetected", stallDetected);
        snapshot.put("overcurrent", overcurrentDetected);
        snapshot.put("reversedEncoder", reversedEncoderDetected);
        snapshot.put("pidSaturated", pidSaturatedDetected);
        snapshot.put("speedError", speedErrorDetected);
        snapshot.put("currentAmps", lastCurrentAmps);
        snapshot.put("faultCount", totalFaultCount);
        snapshot.put("lastPosition", lastPosition);
        snapshot.put("lastPositionChangeMs", lastPositionChangeMs);
        return snapshot;
    }
}
