package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Centralized registry and aggregator for motor errors across all subsystems.
 * <p>
 * Subsystems register their {@link MotorError.FaultSource} (such as
 * {@link MotorFaultDetector}) during initialization, and telemetry viewers can
 * query a unified list of active errors, worst severity, and summary lines.
 */
public final class MotorErrorTracker {
    private static final MotorErrorTracker INSTANCE = new MotorErrorTracker();

    private final List<MotorError.FaultSource> sources = new CopyOnWriteArrayList<>();

    private MotorErrorTracker() { }

    public static MotorErrorTracker getInstance() {
        return INSTANCE;
    }

    /** Clears all registered motor fault sources (call on robot init). */
    public void clear() {
        sources.clear();
    }

    /** Registers a motor fault source to be included in global diagnostics. */
    public void register(MotorError.FaultSource source) {
        if (source != null && !sources.contains(source)) {
            sources.add(source);
        }
    }

    /** Unregisters a motor fault source. */
    public void unregister(MotorError.FaultSource source) {
        if (source != null) {
            sources.remove(source);
        }
    }

    /** Returns reports for every monitored motor. */
    public List<MotorError.MotorErrorReport> getAllReports() {
        List<MotorError.MotorErrorReport> reports = new ArrayList<>(sources.size());
        for (MotorError.FaultSource source : sources) {
            reports.add(source.getReport());
        }
        return reports;
    }

    /** Returns only reports that currently have an active warning or error. */
    public List<MotorError.MotorErrorReport> getActiveErrors() {
        List<MotorError.MotorErrorReport> active = new ArrayList<>();
        for (MotorError.FaultSource source : sources) {
            MotorError.MotorErrorReport report = source.getReport();
            if (report.hasFault()) {
                active.add(report);
            }
        }
        return active;
    }

    /** Returns true if any monitored motor has an active warning or error. */
    public boolean hasAnyError() {
        for (MotorError.FaultSource source : sources) {
            if (source.getReport().hasFault()) {
                return true;
            }
        }
        return false;
    }

    /** Returns true if any monitored motor has a critical (ERROR) fault. */
    public boolean hasCriticalError() {
        for (MotorError.FaultSource source : sources) {
            if (source.getReport().getSeverity() == MotorError.Severity.ERROR) {
                return true;
            }
        }
        return false;
    }

    /** Returns the worst active severity across all monitored motors. */
    public MotorError.Severity getWorstSeverity() {
        MotorError.Severity worst = MotorError.Severity.OK;
        for (MotorError.FaultSource source : sources) {
            MotorError.Severity current = source.getReport().getSeverity();
            if (current.isWorseThan(worst)) {
                worst = current;
            }
        }
        return worst;
    }

    /** Returns the number of monitored motors currently registered. */
    public int getMonitoredMotorCount() {
        return sources.size();
    }

    /** Returns total fault occurrences recorded across all motors this OpMode. */
    public int getTotalFaultCount() {
        int total = 0;
        for (MotorError.FaultSource source : sources) {
            total += source.getReport().getTotalFaultCount();
        }
        return total;
    }

    /** Returns a single-line human-readable summary of motor health. */
    public String getSummaryLine() {
        List<MotorError.MotorErrorReport> active = getActiveErrors();
        if (active.isEmpty()) {
            return "ALL OK (" + sources.size() + " motors)";
        }
        StringBuilder builder = new StringBuilder();
        for (int index = 0; index < active.size(); index++) {
            if (index > 0) {
                builder.append(" | ");
            }
            builder.append(active.get(index).formatCompact());
        }
        return builder.toString();
    }
}
