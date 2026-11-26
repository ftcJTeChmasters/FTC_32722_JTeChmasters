package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.debug.RobotMonitor;
import org.firstinspires.ftc.teamcode.robot.subsystems.Aimer;
import org.firstinspires.ftc.teamcode.util.ButtonUtils;

/**
 * TeleOp mode for driver-controlled operation.
 * <p>
 * Emergency Stop (PlayStation layout):
 * <ul>
 *   <li>Hold both triggers (L2 + R2 > 85% for 150ms): GLOBAL E-STOP (all motors off)</li>
 *   <li>Hold SHARE (0.5s): clear global & subsystem E-Stops</li>
 * </ul>
 * <p>
 * Drive controls depend on {@link RobotConstants#MECANUM_ODO_INSTALLED}:
 * <ul>
 *   <li>Mecanum + Odo installed ({@code true}): Left stick = forward/strafe, Right stick X = turn, Circle = reset odometry</li>
 *   <li>Legacy tank mode ({@code false}): Left stick Y = left drive, Right stick Y = right drive</li>
 * </ul>
 * <p>
 * Mechanism controls:
 * <ul>
 *   <li>Square (X): toggle intake enabled/disabled (or clear intake Auto E-Stop)</li>
 *   <li>Triangle (Y): trigger intake reverse (jam clearing)</li>
 *   <li>R1 (Right Bumper): set drive speed to fast (100%)</li>
 *   <li>L1 (Left Bumper): set drive speed to slow (50%)</li>
 *   <li>Cross (A): toggle flywheel on/off (or clear flywheel Auto E-Stop)</li>
 * </ul>
 * <p>
 * Aimer controls:
 * <ul>
 *   <li>D-Pad UP: set aimer to large ball position (blue/red balls)</li>
 *   <li>D-Pad DOWN: set aimer to small ball position (yellow balls)</li>
 *   <li>D-Pad LEFT: toggle auto-detect on/off</li>
 *   <li>D-Pad RIGHT: toggle team color (BLUE/RED alliance)</li>
 * </ul>
 */
@TeleOp(name = "Driver Controlled")
public class DriverControlled extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);
    private final RobotMonitor monitor = new RobotMonitor(this, robot);
    private ButtonUtils pad1;

    @Override
    public void runOpMode() {
        monitor.init();
        pad1 = new ButtonUtils(gamepad1);

        waitForStart();
        monitor.start();

        while (opModeIsActive()) {
            pad1.update();

            if (!monitor.pollEmergencyStop()) {
                if (RobotConstants.MECANUM_ODO_INSTALLED) {
                    double forward = applyDeadzone(-gamepad1.left_stick_y, RobotConstants.GAMEPAD_DEADZONE);
                    double strafe = applyDeadzone(gamepad1.left_stick_x, RobotConstants.GAMEPAD_DEADZONE);
                    double turn = applyDeadzone(gamepad1.right_stick_x, RobotConstants.GAMEPAD_DEADZONE);
                    robot.driveTrain.setMecanumPowers(forward, strafe, turn);

                    if (pad1.pressed(ButtonUtils.Button.B)) {
                        robot.driveTrain.resetOdometry();
                    }
                } else {
                    double leftPower = applyDeadzone(-gamepad1.left_stick_y, RobotConstants.GAMEPAD_DEADZONE);
                    double rightPower = applyDeadzone(-gamepad1.right_stick_y, RobotConstants.GAMEPAD_DEADZONE);
                    robot.driveTrain.setPowers(leftPower, rightPower);
                }

                // Intake toggle (Square / X)
                if (pad1.pressed(ButtonUtils.Button.X)) {
                    robot.intake.toggle();
                }

                // Emergency reverse for jam clearing (Triangle / Y)
                if (pad1.pressed(ButtonUtils.Button.Y)) {
                    robot.intake.triggerReverse();
                }

                // Speed multiplier: R1 = fast, L1 = slow
                if (pad1.pressed(ButtonUtils.Button.RIGHT_BUMPER)) {
                    robot.driveTrain.setSpeedMultiplier(RobotConstants.FAST_SPEED_MULTIPLIER);
                }
                if (pad1.pressed(ButtonUtils.Button.LEFT_BUMPER)) {
                    robot.driveTrain.setSpeedMultiplier(RobotConstants.SLOW_SPEED_MULTIPLIER);
                }

                // Flywheel toggle (Cross / A)
                if (pad1.pressed(ButtonUtils.Button.A)) {
                    robot.flyWheel.toggle();
                }

                // Aimer controls
                // D-Pad UP: large ball position
                if (pad1.pressed(ButtonUtils.Button.DPAD_UP)) {
                    robot.aimer.setLargeBallPosition();
                }
                // D-Pad DOWN: small ball position
                if (pad1.pressed(ButtonUtils.Button.DPAD_DOWN)) {
                    robot.aimer.setSmallBallPosition();
                }
                // D-Pad LEFT: toggle auto-detect
                if (pad1.pressed(ButtonUtils.Button.DPAD_LEFT)) {
                    robot.aimer.setAutoDetectEnabled(!robot.aimer.isAutoDetectEnabled());
                }
                // D-Pad RIGHT: toggle team color
                if (pad1.pressed(ButtonUtils.Button.DPAD_RIGHT)) {
                    Aimer.setTeamColorBlue(!Aimer.isTeamColorBlue());
                }
            }

            monitor.update();
        }

        robot.driveTrain.stopImmediately();
        robot.flyWheel.stop();
        monitor.stop();
    }

    /**
     * Applies a deadzone to a joystick value and rescales the active range to [0, 1].
     *
     * @param value    raw joystick value (-1.0 to 1.0)
     * @param deadzone deadzone threshold (0.0 to 1.0)
     * @return value with deadzone applied, or 0 if within deadzone
     */
    private static double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        return (value > 0)
                ? (value - deadzone) / (1.0 - deadzone)
                : (value + deadzone) / (1.0 - deadzone);
    }
}