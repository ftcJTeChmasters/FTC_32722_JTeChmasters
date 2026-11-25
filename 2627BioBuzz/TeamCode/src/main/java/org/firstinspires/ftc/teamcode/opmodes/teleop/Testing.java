package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.debug.RobotMonitor;

/**
 * Minimal TeleOp for testing subsystem initialization and telemetry.
 * <p>
 * No driver controls — initializes the robot and streams telemetry to both
 * the Driver Station viewer and the laptop TelemetryServer.
 */
@TeleOp(name = "Testing")
public class Testing extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this);
    private final RobotMonitor monitor = new RobotMonitor(this, robot);

    @Override
    public void runOpMode() {
        monitor.init();

        waitForStart();
        monitor.start();

        while (opModeIsActive()) {
            monitor.update();
        }

        monitor.stop();
    }
}
