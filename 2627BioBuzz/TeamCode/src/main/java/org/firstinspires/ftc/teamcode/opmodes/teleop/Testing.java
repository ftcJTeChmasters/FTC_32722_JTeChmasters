package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;
@TeleOp(name = "Testing")
public class Testing extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();
        RobotConstants constants = new RobotConstants();

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
