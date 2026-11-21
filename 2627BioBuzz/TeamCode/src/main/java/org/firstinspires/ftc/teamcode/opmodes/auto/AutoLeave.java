package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Leave")
public class AutoLeave extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();

        robot.driveTrain.setSpeedMultiplier(1.0);
        robot.driveTrain.setPowers(1.0, 0.2);
        sleep(1000);
        robot.driveTrain.stop();
    }
}