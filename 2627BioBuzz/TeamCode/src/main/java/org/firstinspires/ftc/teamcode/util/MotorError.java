package org.firstinspires.ftc.teamcode.util;

import java.util.Collections;
import java.util.EnumSet;
import java.util.Locale;
import java.util.Set;

/**
 * Pure-logic definitions for motor fault classification, severity, and
 * structured diagnostic reports.
 */
public final class MotorError {
    private MotorError() { }

    /**
     * Categorized motor fault conditions.
     */
    public enum ErrorType {
        /** No active fault. */
        NONE(Severity.OK, "OK"),
        /** Motor object is null or missing from hardware configuration. */
        HARDWARE_MISSING(Severity.ERROR, "MISSING"),
        /** Motor is powered but encoder ticks never change (unplugged encoder). */
        ENCODER_FAULT(Severity.ERROR, "ENC_FAULT"),
        /** Motor is powered strongly but speed remains below stall threshold. */
        STALLED(Severity.ERROR, "STALLED"),
        /** Motor current draw exceeds safe threshold (DcMotorEx). */
        OVERCURRENT(Severity.ERROR, "OVERCURRENT"),
        /** Encoder counts opposite to commanded motor power direction. */
        REVERSED_ENCODER(Severity.WARNING, "REV_ENC"),
        /** Velocity/position PID output is pinned at saturation limit. */
        PID_SATURATED(Severity.WARNING, "PID_SAT"),
        /** Actual speed deviates from target speed beyond allowed tolerance. */
        SPEED_ERROR(Severity.WARNING, "SPD_ERR");

        private final Severity severity;
        private final String shortLabel;

        ErrorType(Severity severity, String shortLabel) {
            this.severity = severity;
            this.shortLabel = shortLabel;
        }

        public Severity getSeverity() {
            return severity;
        }

        public String getShortLabel() {
            return shortLabel;
        }
    }

    /**
     * Diagnostic severity levels.
     */
    public enum Severity {
        OK(0),
        WARNING(1),
        ERROR(2);

        private final int rank;

        Severity(int rank) {
            this.rank = rank;
        }

        public int getRank() {
            return rank;
        }

        public boolean isWorseThan(Severity other) {
            return this.rank > other.rank;
        }
    }

    /**
     * Interface implemented by motor fault monitors so trackers and telemetry
     * viewers remain decoupled from hardware types.
     */
    public interface FaultSource {
        MotorErrorReport getReport();
    }

    /**
     * Immutable diagnostic snapshot for a single monitored motor.
     */
    public static final class MotorErrorReport {
        private final String subsystem;
        private final String motorName;
        private final ErrorType primaryError;
        private final Set<ErrorType> activeErrors;
        private final Severity severity;
        private final String message;
        private final double lastPower;
        private final double lastSpeedTicksPerSec;
        private final double lastCurrentAmps;
        private final int totalFaultCount;
        private final long timestampMs;

        public MotorErrorReport(
                String subsystem,
                String motorName,
                ErrorType primaryError,
                Set<ErrorType> activeErrors,
                Severity severity,
                String message,
                double lastPower,
                double lastSpeedTicksPerSec,
                double lastCurrentAmps,
                int totalFaultCount,
                long timestampMs
        ) {
            this.subsystem = subsystem == null ? "unknown" : subsystem;
            this.motorName = motorName == null ? "motor" : motorName;
            this.primaryError = primaryError == null ? ErrorType.NONE : primaryError;
            this.activeErrors = activeErrors == null || activeErrors.isEmpty()
                    ? Collections.emptySet()
                    : Collections.unmodifiableSet(EnumSet.copyOf(activeErrors));
            this.severity = severity == null ? Severity.OK : severity;
            this.message = message == null ? "OK" : message;
            this.lastPower = lastPower;
            this.lastSpeedTicksPerSec = lastSpeedTicksPerSec;
            this.lastCurrentAmps = lastCurrentAmps;
            this.totalFaultCount = totalFaultCount;
            this.timestampMs = timestampMs;
        }

        public String getSubsystem() {
            return subsystem;
        }

        public String getMotorName() {
            return motorName;
        }

        public ErrorType getPrimaryError() {
            return primaryError;
        }

        public Set<ErrorType> getActiveErrors() {
            return activeErrors;
        }

        public Severity getSeverity() {
            return severity;
        }

        public String getMessage() {
            return message;
        }

        public double getLastPower() {
            return lastPower;
        }

        public double getLastSpeedTicksPerSec() {
            return lastSpeedTicksPerSec;
        }

        public double getLastCurrentAmps() {
            return lastCurrentAmps;
        }

        public int getTotalFaultCount() {
            return totalFaultCount;
        }

        public long getTimestampMs() {
            return timestampMs;
        }

        public boolean hasFault() {
            return primaryError != ErrorType.NONE;
        }

        public String formatCompact() {
            if (!hasFault()) {
                return String.format(Locale.US, "%s/%s: OK", subsystem, motorName);
            }
            return String.format(
                    Locale.US,
                    "%s/%s [%s] %s",
                    subsystem,
                    motorName,
                    primaryError.getShortLabel(),
                    message
            );
        }
    }
}
