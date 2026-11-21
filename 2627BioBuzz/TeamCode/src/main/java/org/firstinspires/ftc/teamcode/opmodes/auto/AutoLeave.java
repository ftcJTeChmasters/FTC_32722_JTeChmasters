package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.NonBlockingWait;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Leave")
public class AutoLeave extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    NonBlockingWait wait = new NonBlockingWait();
    ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();
        robot.driveTrain.setPowers(0.2, 0.2);
        
        stateTimer.reset();

        // TODO: autonomous sequence
        wait.start(1000);
        robot.driveTrain.setPowers(-0.2, -0.2);
        while (!wait.isDone()) {
            robot.updateAll();
            if (wait.isDone()) {
                robot.driveTrain.stop();
                break;
            }
        }
        robot.driveTrain.stopImmediately();
    
    }
}
