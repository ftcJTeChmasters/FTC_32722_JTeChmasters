package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.util.ButtonUtils;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@TeleOp(name = "Driver Controlled")
public class DriverControlled extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);
    private final ElapsedTime stateTimer = new ElapsedTime();
    private ButtonUtils pad1;
    private Servo ServoBall;    

    @Override
    public void runOpMode() {
        robot.init();
        RobotConstants consstants = new RobotConstants();
        pad1 = new ButtonUtils(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        stateTimer.reset();
        TelemetryServer.getInstance().setStarted(true);
        
        while (opModeIsActive()) {
            
            robot.updateAll();
            robot.driveTrain.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y);
            if (pad1.pressed(ButtonUtils.Button.X)) {  // if SQUARE is pressed, toggle intake enabled/disabled and reset state to idle
                robot.intake.enabled = !robot.intake.enabled;
                robot.intake.currentState = RobotConstants.INTAKE_MOTOR_IDLE;
            }
            if (pad1.pressed(ButtonUtils.Button.Y)) {  // if TRIANGLE is pressed, emergency overwrite reverse intake direction (for clearing jams)
                robot.intake.currentState = RobotConstants.INTAKE_MOTOR_REVERSE;
            }
            if (pad1.pressed(ButtonUtils.Button.RIGHT_BUMPER)) { // if RIGHT BUMPER is pressed, set speed multiplier to 1.0 (full speed)
                robot.driveTrain.setSpeedMultiplier(RobotConstants.FAST_SPEED_MULTIPLIER);
            }
            if (pad1.pressed(ButtonUtils.Button.LEFT_BUMPER)) { // if LEFT BUMPER is pressed, set speed multiplier to 0.2 (low speed)
                robot.driveTrain.stopImmediately();
                robot.driveTrain.setSpeedMultiplier(RobotConstants.SLOW_SPEED_MULTIPLIER);
            }
            if (pad1.pressed(ButtonUtils.Button.A)) { // if CROSS is pressed, toggle flywheel on/off
                robot.flyWheel.toggle();
            } 
        }

        TelemetryServer.getInstance().setStarted(false);
        TelemetryServer.getInstance().setCurrentTask("Stopped");
    }
}