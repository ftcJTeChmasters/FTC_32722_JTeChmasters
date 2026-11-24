package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.NonBlockingWait;

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
        robot.driveTrain.setPowers(0.2, 0.2);

        // Wait 1 second
        wait.start(1000);
        while (!wait.isDone() && opModeIsActive()) {
            robot.updateAll();
        }

        // Drive backward slowly
        robot.driveTrain.setPowers(-0.2, -0.2);
        wait.start(1000);
        while (!wait.isDone() && opModeIsActive()) {
            robot.updateAll();
        }

        robot.driveTrain.stopImmediately();
    }
}
