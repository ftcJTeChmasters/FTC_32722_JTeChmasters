package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Minimal TeleOp for testing subsystem initialization and telemetry.
 * <p>
 * No driver controls — just initializes the robot and streams telemetry
 * to the TelemetryServer for dashboard debugging.
 */
@TeleOp(name = "Testing")
public class Testing extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        TelemetryServer.getInstance().setStarted(true);

        while (opModeIsActive()) {
            robot.updateAll();
        }

        TelemetryServer.getInstance().setStarted(false);
        TelemetryServer.getInstance().setCurrentTask("Stopped");
    }
}
