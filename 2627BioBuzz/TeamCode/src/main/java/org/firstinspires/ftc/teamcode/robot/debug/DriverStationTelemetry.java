package org.firstinspires.ftc.teamcode.robot.debug;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.odometry.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.EmergencyStop;
import org.firstinspires.ftc.teamcode.util.MotorError;
import org.firstinspires.ftc.teamcode.util.MotorErrorTracker;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Formatted, color-coded telemetry viewer for the FTC Driver Station screen.
 * <p>
 * Renders robot state, Emergency Stop / Auto E-Stop banners, battery voltage,
 * motor error alerts (from {@link MotorErrorTracker}), drivetrain/odometry pose,
 * flywheel velocity, and intake state directly on the Driver Station using HTML.
 */
public class DriverStationTelemetry {

    private static final String COLOR_BRAND = "#38BDF8";
    private static final String COLOR_OK = "#22C55E";
    private static final String COLOR_WARN = "#FACC15";
    private static final String COLOR_ERROR = "#EF4444";
    private static final String COLOR_MUTED = "#94A3B8";
    private static final String COLOR_DRIVE = "#A78BFA";
    private static final String COLOR_FLYWHEEL = "#FB923C";
    private static final String COLOR_INTAKE = "#34D399";

    private static final double PERCENT_SCALE = 100.0;
    private static final int SECONDS_PER_MINUTE = 60;

    private final Telemetry telemetry;
    private final String opModeName;
    private final Map<String, String> extras = new LinkedHashMap<>();

    private String robotState = "WAITING";
    private String currentTask = "";

    public DriverStationTelemetry(Telemetry telemetry, String opModeName) {
        this.telemetry = telemetry;
        this.opModeName = opModeName == null ? "OpMode" : opModeName;
        configureTelemetry();
    }

    private void configureTelemetry() {
        telemetry.setMsTransmissionInterval(RobotConstants.DS_TELEMETRY_INTERVAL_MS);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

    public void setRobotState(String state) {
        this.robotState = state == null ? "WAITING" : state;
    }

    public void setCurrentTask(String task) {
        this.currentTask = task == null ? "" : task;
    }

    public void setExtra(String key, Object value) {
        if (key == null) return;
        if (value == null) {
            extras.remove(key);
        } else {
            extras.put(key, String.valueOf(value));
        }
    }

    public void clearExtras() {
        extras.clear();
    }

    /**
     * Builds and pushes the full Driver Station dashboard frame.
     */
    public void render(
            double runtimeSeconds,
            double batteryVoltage,
            DriveTrain driveTrain,
            FlyWheel flyWheel,
            Intake intake
    ) {
        // 0. Emergency Stop Banner (highest visibility when active)
        renderEmergencyStops();

        // 1. Header bar
        String stateColor = getStateColor(robotState);
        telemetry.addLine(String.format(
                Locale.US,
                "<b><font color=\"%s\">BioBuzz 32722</font></b>  |  <b>%s</b>  |  <font color=\"%s\"><b>%s</b></font>",
                COLOR_BRAND,
                escapeHtml(opModeName),
                stateColor,
                escapeHtml(robotState)
        ));

        // 2. Battery, runtime & drivetrain mode bar
        String batteryColor = getBatteryColor(batteryVoltage);
        String batteryText = batteryVoltage > 0
                ? String.format(Locale.US, "%.2fV", batteryVoltage)
                : "--V";
        String driveModeLabel = (driveTrain != null && driveTrain.isMecanumInstalled())
                ? "Mecanum+Odo"
                : "Tank (Legacy)";

        telemetry.addLine(String.format(
                Locale.US,
                "Bat: <font color=\"%s\"><b>%s</b></font>  |  Time: <b>%s</b>  |  Drive: <b>%s</b>",
                batteryColor,
                batteryText,
                formatRuntime(runtimeSeconds),
                driveModeLabel
        ));

        if (!currentTask.isEmpty()) {
            telemetry.addLine(String.format(
                    Locale.US,
                    "<font color=\"%s\">Task:</font> <b>%s</b>",
                    COLOR_MUTED,
                    escapeHtml(currentTask)
            ));
        }

        // 3. Motor Errors banner
        renderMotorErrors();

        // 4. DriveTrain & Odometry
        renderDriveTrain(driveTrain);

        // 5. FlyWheel
        renderFlyWheel(flyWheel);

        // 6. Intake
        renderIntake(intake);

        // 7. OpMode-specific extras
        if (!extras.isEmpty()) {
            StringBuilder extraLine = new StringBuilder();
            extraLine.append("<b><font color=\"").append(COLOR_WARN).append("\">▸ Info</font></b>");
            for (Map.Entry<String, String> entry : extras.entrySet()) {
                extraLine.append("  |  ")
                        .append(escapeHtml(entry.getKey()))
                        .append(": <b>")
                        .append(escapeHtml(entry.getValue()))
                        .append("</b>");
            }
            telemetry.addLine(extraLine.toString());
        }

        telemetry.update();
    }

    private void renderEmergencyStops() {
        EmergencyStop eStop = EmergencyStop.getInstance();
        if (!eStop.hasAnyStop()) {
            return;
        }

        if (eStop.isGlobalEStopped()) {
            telemetry.addLine(String.format(
                    Locale.US,
                    "<font color=\"%s\"><b>🛑 GLOBAL E-STOP ACTIVE:</b> %s</font>",
                    COLOR_ERROR,
                    escapeHtml(eStop.getGlobalReason())
            ));
        }

        Map<String, String> subsystemStops = eStop.getActiveSubsystemEStops();
        for (Map.Entry<String, String> entry : subsystemStops.entrySet()) {
            telemetry.addLine(String.format(
                    Locale.US,
                    "<font color=\"%s\"><b>⛔ AUTO E-STOP [%s]:</b></font> %s",
                    COLOR_ERROR,
                    escapeHtml(entry.getKey()),
                    escapeHtml(entry.getValue())
            ));
        }

        double resetProgress = eStop.getResetHoldProgress();
        if (resetProgress > 0.0) {
            int pct = (int) Math.round(resetProgress * PERCENT_SCALE);
            telemetry.addLine(String.format(
                    Locale.US,
                    "<font color=\"%s\"><b>Resetting E-Stop... %d%%</b></font>",
                    COLOR_WARN,
                    pct
            ));
        } else {
            telemetry.addLine(String.format(
                    Locale.US,
                    "<font color=\"%s\">Hold BACK (0.5s) to clear all E-Stops</font>",
                    COLOR_WARN
            ));
        }
    }

    private void renderMotorErrors() {
        MotorErrorTracker tracker = MotorErrorTracker.getInstance();
        List<MotorError.MotorErrorReport> activeErrors = tracker.getActiveErrors();

        if (activeErrors.isEmpty()) {
            telemetry.addLine(String.format(
                    Locale.US,
                    "<font color=\"%s\"><b>✓ Motors:</b> ALL OK</font> <font color=\"%s\">(%d monitored)</font>",
                    COLOR_OK,
                    COLOR_MUTED,
                    tracker.getMonitoredMotorCount()
            ));
            return;
        }

        String headerColor = tracker.hasCriticalError() ? COLOR_ERROR : COLOR_WARN;
        telemetry.addLine(String.format(
                Locale.US,
                "<font color=\"%s\"><b>⚠ MOTOR ALERTS (%d active, %d total)</b></font>",
                headerColor,
                activeErrors.size(),
                tracker.getTotalFaultCount()
        ));

        for (MotorError.MotorErrorReport report : activeErrors) {
            String errColor = report.getSeverity() == MotorError.Severity.ERROR ? COLOR_ERROR : COLOR_WARN;
            telemetry.addLine(String.format(
                    Locale.US,
                    "  <font color=\"%s\"><b>[%s]</b></font> <b>%s/%s</b>: %s",
                    errColor,
                    escapeHtml(report.getPrimaryError().getShortLabel()),
                    escapeHtml(report.getSubsystem()),
                    escapeHtml(report.getMotorName()),
                    escapeHtml(report.getMessage())
            ));
        }
    }

    private void renderDriveTrain(DriveTrain driveTrain) {
        if (driveTrain == null) return;

        int speedPct = (int) Math.round(driveTrain.getSpeedMultiplier() * PERCENT_SCALE);
        String speedTag = driveTrain.getSpeedMultiplier() >= RobotConstants.FAST_SPEED_MULTIPLIER
                ? "FAST"
                : "SLOW";
        String eStopTag = driveTrain.isEStopped()
                ? String.format(Locale.US, "  |  <font color=\"%s\"><b>[E-STOP]</b></font>", COLOR_ERROR)
                : "";

        telemetry.addLine(String.format(
                Locale.US,
                "<b><font color=\"%s\">▸ DriveTrain</font></b>  |  Speed: <b>%d%% (%s)</b>%s",
                COLOR_DRIVE,
                speedPct,
                speedTag,
                eStopTag
        ));

        if (driveTrain.isMecanumInstalled()) {
            TwoWheelOdometry odometry = driveTrain.getOdometry();
            if (odometry != null) {
                telemetry.addLine(String.format(
                        Locale.US,
                        "  Pose: X=<b>%.1f\"</b>  Y=<b>%.1f\"</b>  Hdg=<b>%.1f°</b>  |  Pods: F=<b>%d</b> S=<b>%d</b>",
                        odometry.getXInches(),
                        odometry.getYInches(),
                        odometry.getHeadingDegrees(),
                        odometry.getForwardTicks(),
                        odometry.getStrafeTicks()
                ));
            }
            telemetry.addLine(String.format(
                    Locale.US,
                    "  Pwr: FL <b>%+.2f</b> | FR <b>%+.2f</b> | BL <b>%+.2f</b> | BR <b>%+.2f</b>",
                    driveTrain.getFrontLeftPower(),
                    driveTrain.getFrontRightPower(),
                    driveTrain.getBackLeftPower(),
                    driveTrain.getBackRightPower()
            ));
        } else {
            telemetry.addLine(String.format(
                    Locale.US,
                    "  L: <b>%+.2f</b> (<b>%.0f</b> t/s)  |  R: <b>%+.2f</b> (<b>%.0f</b> t/s)",
                    driveTrain.getLeftPower(),
                    driveTrain.getLastLeftSpeed(),
                    driveTrain.getRightPower(),
                    driveTrain.getLastRightSpeed()
            ));
        }
    }

    private void renderFlyWheel(FlyWheel flyWheel) {
        if (flyWheel == null) return;

        boolean eStopped = flyWheel.isEStopped();
        boolean running = flyWheel.isRunning();
        String statusText = eStopped ? "E-STOP" : (running ? "ON" : "OFF");
        String statusColor = eStopped ? COLOR_ERROR : (running ? COLOR_OK : COLOR_MUTED);

        StringBuilder flags = new StringBuilder();
        if (flyWheel.isPidSaturated()) {
            flags.append(" <font color=\"").append(COLOR_WARN).append("\">[SAT]</font>");
        }
        if (flyWheel.isSpeedErrorExceeded()) {
            flags.append(" <font color=\"").append(COLOR_WARN).append("\">[SPD]</font>");
        }

        telemetry.addLine(String.format(
                Locale.US,
                "<b><font color=\"%s\">▸ FlyWheel</font></b>  |  <font color=\"%s\"><b>%s</b></font>  |  <b>%.0f</b> / <b>%.0f</b> t/s  |  Pwr: <b>%.2f</b>%s",
                COLOR_FLYWHEEL,
                statusColor,
                statusText,
                flyWheel.getCurrentSpeed(),
                flyWheel.getTargetSpeed(),
                flyWheel.getMotorPower(),
                flags.toString()
        ));
    }

    private void renderIntake(Intake intake) {
        if (intake == null) return;

        boolean eStopped = intake.isEStopped();
        String enabledText = eStopped ? "E-STOP" : (intake.enabled ? "ON" : "OFF");
        String enabledColor = eStopped ? COLOR_ERROR : (intake.enabled ? COLOR_OK : COLOR_MUTED);
        String recoverySuffix = intake.getRecoveryStage() > 0
                ? String.format(Locale.US, "  |  <font color=\"%s\">Recov:%d</font>", COLOR_WARN, intake.getRecoveryStage())
                : "";

        telemetry.addLine(String.format(
                Locale.US,
                "<b><font color=\"%s\">▸ Intake</font></b>  |  <font color=\"%s\"><b>%s</b></font>  |  State: <b>%s</b>  |  Vel: <b>%.0f</b> t/s%s",
                COLOR_INTAKE,
                enabledColor,
                enabledText,
                escapeHtml(intake.getStateName()),
                intake.getLastSpeed(),
                recoverySuffix
        ));
    }

    private static String getStateColor(String state) {
        if ("E-STOP".equalsIgnoreCase(state)) return COLOR_ERROR;
        if ("RUNNING".equalsIgnoreCase(state)) return COLOR_OK;
        if ("WAITING".equalsIgnoreCase(state)) return COLOR_WARN;
        return COLOR_MUTED;
    }

    private static String getBatteryColor(double voltage) {
        if (voltage <= 0) return COLOR_MUTED;
        if (voltage >= RobotConstants.DS_GOOD_BATTERY_VOLTAGE) return COLOR_OK;
        if (voltage >= RobotConstants.DS_WARN_BATTERY_VOLTAGE) return COLOR_WARN;
        return COLOR_ERROR;
    }

    private static String formatRuntime(double runtimeSeconds) {
        int totalSeconds = (int) Math.max(0, Math.floor(runtimeSeconds));
        int minutes = totalSeconds / SECONDS_PER_MINUTE;
        double remainderSeconds = Math.max(0.0, runtimeSeconds - (minutes * SECONDS_PER_MINUTE));
        return String.format(Locale.US, "%d:%04.1f", minutes, remainderSeconds);
    }

    private static String escapeHtml(String text) {
        if (text == null) return "";
        return text.replace("&", "&amp;")
                .replace("<", "&lt;")
                .replace(">", "&gt;");
    }
}
