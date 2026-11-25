package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.debug.RobotMonitor;
import org.firstinspires.ftc.teamcode.util.NonBlockingWait;

/**
 * Simple autonomous that drives forward, waits, then drives backward.
 */
@Autonomous(name = "Auto Leave")
public class AutoLeave extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this);
    private final RobotMonitor monitor = new RobotMonitor(this, robot);
    private final NonBlockingWait wait = new NonBlockingWait();

    @Override
    public void runOpMode() {
        monitor.init();
        waitForStart();
        monitor.start();

        // Drive forward slowly
        monitor.setCurrentTask("Driving forward");
        robot.driveTrain.setPowers(
                RobotConstants.AUTO_LEAVE_DRIVE_POWER,
                RobotConstants.AUTO_LEAVE_DRIVE_POWER
        );

        wait.start(RobotConstants.AUTO_LEAVE_WAIT_MILLISECONDS);
        while (!wait.isDone() && opModeIsActive()) {
            monitor.update();
        }

        // Drive backward slowly
        monitor.setCurrentTask("Driving backward");
        robot.driveTrain.setPowers(
                -RobotConstants.AUTO_LEAVE_DRIVE_POWER,
                -RobotConstants.AUTO_LEAVE_DRIVE_POWER
        );
        wait.start(RobotConstants.AUTO_LEAVE_WAIT_MILLISECONDS);
        while (!wait.isDone() && opModeIsActive()) {
            monitor.update();
        }

        robot.driveTrain.stopImmediately();
        monitor.stop();
    }
}
