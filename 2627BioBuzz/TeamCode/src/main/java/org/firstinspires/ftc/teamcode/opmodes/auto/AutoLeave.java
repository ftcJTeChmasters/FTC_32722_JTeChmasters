package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.NonBlockingWait;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Simple autonomous that drives forward, waits, then drives backward.
 * <p>
 * Placeholder for more complex autonomous routines.
 */
@Autonomous(name = "Auto Leave")
public class AutoLeave extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this);
    private final NonBlockingWait wait = new NonBlockingWait();

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();

        // Drive forward slowly
        robot.driveTrain.setPowers(
                RobotConstants.AUTO_LEAVE_DRIVE_POWER,
                RobotConstants.AUTO_LEAVE_DRIVE_POWER
        );

        // Wait
        wait.start(RobotConstants.AUTO_LEAVE_WAIT_MILLISECONDS);
        while (!wait.isDone() && opModeIsActive()) {
            robot.updateAll();
        }

        // Drive backward slowly
        robot.driveTrain.setPowers(
                -RobotConstants.AUTO_LEAVE_DRIVE_POWER,
                -RobotConstants.AUTO_LEAVE_DRIVE_POWER
        );
        wait.start(RobotConstants.AUTO_LEAVE_WAIT_MILLISECONDS);
        while (!wait.isDone() && opModeIsActive()) {
            robot.updateAll();
        }

        robot.driveTrain.stopImmediately();
    }
}
