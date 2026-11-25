package org.firstinspires.ftc.teamcode.robot.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.EmergencyStop;
import org.firstinspires.ftc.teamcode.util.MotorErrorTracker;
import org.firstinspires.ftc.teamcode.util.MotorUtils;

/**
 * Companion runtime monitor for {@link RobotHardware} that manages:
 * <ul>
 *   <li>Global Emergency Stop polling and reset ({@link EmergencyStop})</li>
 *   <li>Centralized motor error publishing ({@link MotorErrorTracker})</li>
 *   <li>Driver Station HUD rendering ({@link DriverStationTelemetry})</li>
 * </ul>
 * <p>
 * Keeps {@link RobotHardware} untouched so FTC Code Utils can regenerate it
 * freely without losing Driver Station telemetry or E-Stop functionality.
 */
public class RobotMonitor {
    private final LinearOpMode opMode;
    private final RobotHardware robot;
    private VoltageSensor voltageSensor;
    private DriverStationTelemetry dsTelemetry;
    private double lastBatteryVoltage = 0.0;

    public RobotMonitor(LinearOpMode opMode, RobotHardware robot) {
        this.opMode = opMode;
        this.robot = robot;
    }

    /**
     * Clears stale error/E-Stop state, initializes {@link RobotHardware},
     * and renders the initial Driver Station telemetry frame.
     */
    public void init() {
        EmergencyStop.getInstance().resetAll();
        MotorErrorTracker.getInstance().clear();
        MotorUtils.clearAllMotorStates();

        robot.init();

        for (VoltageSensor sensor : opMode.hardwareMap.voltageSensor) {
            voltageSensor = sensor;
            lastBatteryVoltage = sensor.getVoltage();
            break;
        }

        String opModeName = opMode.getClass().getSimpleName();
        TelemetryServer.getInstance().setStarted(false);

        dsTelemetry = new DriverStationTelemetry(opMode.telemetry, opModeName);
        dsTelemetry.setRobotState("WAITING");
        dsTelemetry.render(
                opMode.getRuntime(),
                lastBatteryVoltage,
                robot.driveTrain,
                robot.flyWheel,
                robot.intake
        );
    }

    /** Marks the OpMode as started in both TelemetryServer and DriverStationTelemetry. */
    public void start() {
        TelemetryServer.getInstance().setStarted(true);
        if (dsTelemetry != null) {
            dsTelemetry.setRobotState("RUNNING");
        }
    }

    /** Marks the OpMode as stopped and renders one final Driver Station frame. */
    public void stop() {
        TelemetryServer.getInstance().setStarted(false);
        TelemetryServer.getInstance().setCurrentTask("Stopped");
        if (dsTelemetry != null) {
            dsTelemetry.setRobotState("STOPPED");
            dsTelemetry.setCurrentTask("Stopped");
            dsTelemetry.render(
                    opMode.getRuntime(),
                    lastBatteryVoltage,
                    robot.driveTrain,
                    robot.flyWheel,
                    robot.intake
            );
        }
    }

    /** Updates the current task shown on both the Driver Station and TelemetryServer. */
    public void setCurrentTask(String task) {
        TelemetryServer.getInstance().setCurrentTask(task);
        if (dsTelemetry != null) {
            dsTelemetry.setCurrentTask(task);
        }
    }

    /** Adds or updates a custom key/value line on the Driver Station telemetry viewer. */
    public void setDsExtra(String key, Object value) {
        if (dsTelemetry != null) {
            dsTelemetry.setExtra(key, value);
        }
    }

    public DriverStationTelemetry getDriverStationTelemetry() {
        return dsTelemetry;
    }

    /**
     * Polls both gamepads for global E-Stop trigger/reset gestures and enforces an
     * immediate power cutoff if global E-Stop is active.
     *
     * @return true if global E-Stop is currently active
     */
    public boolean pollEmergencyStop() {
        EmergencyStop eStop = EmergencyStop.getInstance();
        eStop.updateGamepadInputs(
                opMode.gamepad1.left_trigger,
                opMode.gamepad1.right_trigger,
                opMode.gamepad1.back,
                opMode.gamepad2.left_trigger,
                opMode.gamepad2.right_trigger,
                opMode.gamepad2.back
        );

        if (eStop.isGlobalEStopped()) {
            robot.driveTrain.stopImmediately();
            robot.flyWheel.stop();
            robot.intake.setSpeed(0.0);
            if (dsTelemetry != null) {
                dsTelemetry.setRobotState("E-STOP");
            }
            TelemetryServer.getInstance().setRobotState("estop");
            return true;
        }

        if (dsTelemetry != null && opMode.opModeIsActive()) {
            dsTelemetry.setRobotState("RUNNING");
            TelemetryServer.getInstance().setRobotState("running");
        }
        return false;
    }

    /**
     * Polls E-Stop, calls {@link RobotHardware#updateAll()}, publishes motor error
     * and E-Stop snapshots to {@link TelemetryServer}, and renders the Driver
     * Station telemetry viewer. Call once per loop.
     */
    public void update() {
        pollEmergencyStop();
        robot.updateAll();

        if (voltageSensor != null) {
            lastBatteryVoltage = voltageSensor.getVoltage();
        }

        double runtime = opMode.getRuntime();
        EmergencyStop eStop = EmergencyStop.getInstance();
        TelemetryServer server = TelemetryServer.getInstance();
        server.setMotorErrors(MotorErrorTracker.getInstance().getActiveErrors());
        server.setEmergencyStop(
                eStop.isGlobalEStopped(),
                eStop.getGlobalReason(),
                eStop.getActiveSubsystemEStops()
        );

        if (dsTelemetry != null) {
            dsTelemetry.render(
                    runtime,
                    lastBatteryVoltage,
                    robot.driveTrain,
                    robot.flyWheel,
                    robot.intake
            );
        }
    }
}
