package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.util.ButtonUtils;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * TeleOp mode for driver-controlled operation.
 * <p>
 * Controls:
 * <ul>
 *   <li>Left stick Y: left drive motor power</li>
 *   <li>Right stick Y: right drive motor power</li>
 *   <li>X (Square): toggle intake enabled/disabled</li>
 *   <li>Y (Triangle): force intake into reverse (jam clearing)</li>
 *   <li>Right Bumper: set drive speed to fast (100%)</li>
 *   <li>Left Bumper: set drive speed to slow (50%)</li>
 *   <li>A (Cross): toggle flywheel on/off</li>
 * </ul>
 */
@TeleOp(name = "Driver Controlled")
public class DriverControlled extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);
    private final ElapsedTime stateTimer = new ElapsedTime();
    private ButtonUtils pad1;

    @Override
    public void runOpMode() {
        robot.init();
        pad1 = new ButtonUtils(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        stateTimer.reset();
        TelemetryServer.getInstance().setStarted(true);

        while (opModeIsActive()) {
            // Update button states at start of loop
            pad1.update();

            // Update all subsystems (PID loops, telemetry streaming)
            robot.updateAll();

            // Drive control - tank drive with deadzone
            double leftPower = applyDeadzone(gamepad1.left_stick_y, RobotConstants.GAMEPAD_DEADZONE);
            double rightPower = applyDeadzone(gamepad1.right_stick_y, RobotConstants.GAMEPAD_DEADZONE);
            robot.driveTrain.setPowers(leftPower, rightPower);

            // Intake toggle (X / Square)
            if (pad1.pressed(ButtonUtils.Button.X)) {
                robot.intake.enabled = !robot.intake.enabled;
                robot.intake.currentState = RobotConstants.INTAKE_MOTOR_IDLE;
            }

            // Emergency reverse for jam clearing (Y / Triangle)
            if (pad1.pressed(ButtonUtils.Button.Y)) {
                robot.intake.currentState = RobotConstants.INTAKE_MOTOR_REVERSE;
            }

            // Speed multiplier: Right Bumper = fast, Left Bumper = slow
            if (pad1.pressed(ButtonUtils.Button.RIGHT_BUMPER)) {
                robot.driveTrain.setSpeedMultiplier(RobotConstants.FAST_SPEED_MULTIPLIER);
            }
            if (pad1.pressed(ButtonUtils.Button.LEFT_BUMPER)) {
                robot.driveTrain.setSpeedMultiplier(RobotConstants.SLOW_SPEED_MULTIPLIER);
            }

            // Flywheel toggle (A / Cross)
            if (pad1.pressed(ButtonUtils.Button.A)) {
                robot.flyWheel.toggle();
            }
        }

        TelemetryServer.getInstance().setStarted(false);
        TelemetryServer.getInstance().setCurrentTask("Stopped");
    }

    /**
     * Applies a deadzone to a joystick value.
     *
     * @param value     raw joystick value (-1.0 to 1.0)
     * @param deadzone  deadzone threshold (0.0 to 1.0)
     * @return value with deadzone applied, or 0 if within deadzone
     */
    private static double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        // Rescale to maintain full range
        return (value > 0)
                ? (value - deadzone) / (1.0 - deadzone)
                : (value + deadzone) / (1.0 - deadzone);
    }
}