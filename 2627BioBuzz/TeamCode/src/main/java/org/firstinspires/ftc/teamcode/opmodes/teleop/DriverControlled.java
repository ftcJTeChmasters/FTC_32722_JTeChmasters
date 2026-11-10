package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.util.ButtonUtils;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@TeleOp(name = "Driver Controlled")
public class DriverControlled extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);
    private final ElapsedTime stateTimer = new ElapsedTime();
    private ButtonUtils pad1;

    @Override
    public void runOpMode() {
        robot.init();
        RobotConstants constants = new RobotConstants();
        pad1 = new ButtonUtils(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        stateTimer.reset();
        TelemetryServer.getInstance().setStarted(true);
        
        while (opModeIsActive()) {
            robot.updateAll();
            if (pad1.pressed(ButtonUtils.Button.X)) {
                robot.intake.enabled = !robot.intake.enabled;
            }
        }

        TelemetryServer.getInstance().setStarted(false);
        TelemetryServer.getInstance().setCurrentTask("Stopped");
    }
}