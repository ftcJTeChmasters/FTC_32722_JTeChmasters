package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;

/**
 * FlyWheel subsystem with PID velocity control and fault detection.
 * <p>
 * Uses a PID controller to maintain a target velocity. Call {@link #update()}
 * once per loop from the OpMode.
 * <p>
 * Fault detection includes:
 * <ul>
 *   <li>Motor stall (powered but not spinning)</li>
 *   <li>Encoder faults</li>
 *   <li>PID output saturation</li>
 *   <li>Speed error threshold exceeded</li>
 * </ul>
 */
public class FlyWheel {

    private final DcMotor flyWheelMotor;
    private final MotorUtils motorUtils = new MotorUtils();
    private final PIDController pid = new PIDController(
            RobotConstants.FLYWHEEL_P,
            RobotConstants.FLYWHEEL_I,
            RobotConstants.FLYWHEEL_D,
            RobotConstants.FLYWHEEL_F
    );
    private double targetSpeed = RobotConstants.FLYWHEEL_DEFAULT_TARGET_SPEED;
    private boolean running = false;

    // Fault detection
    private long lastPositionChangeMilliseconds = 0;
    private int lastPosition = 0;
    private boolean stallDetected = false;
    private boolean encoderFault = false;
    private boolean pidSaturated = false;
    private boolean speedErrorExceeded = false;

    public FlyWheel(HardwareMap hardwareMap) {
        flyWheelMotor = hardwareMap.get(DcMotor.class, HardwareNames.FLY_WHEEL_MOTOR);

        // Clear any stale velocity state from previous OpMode runs
        MotorUtils.clearMotorState(flyWheelMotor);

        // Initialize fault detection
        lastPosition = flyWheelMotor.getCurrentPosition();
        lastPositionChangeMilliseconds = System.currentTimeMillis();

        TelemetryServer.getInstance().setTargetedPidListener((target, p, i, d, f, setpoint) -> {
            if ("FlyWheel".equals(target)) {
                pid.setPIDF(p, i, d, f);
                if (setpoint != null) {
                    targetSpeed = setpoint;
                }
            }
        });
    }

    /**
     * Starts the flywheel at the given target speed.
     *
     * @param targetSpeed target velocity in encoder ticks per second
     */
    public void start(double targetSpeed) {
        running = true;
        this.targetSpeed = targetSpeed;
        pid.reset(); // Reset PID when starting fresh
    }

    /** Starts the flywheel at the default target speed from RobotConstants. */
    public void start() {
        start(RobotConstants.FLYWHEEL_DEFAULT_TARGET_SPEED);
    }

    /** Stops the flywheel. */
    public void stop() {
        running = false;
    }

    /** Toggles the flywheel on/off. */
    public void toggle() {
        running = !running;
        if (running) {
            pid.reset();
        }
    }

    /**
     * Call this once per loop from the OpMode.
     * <p>
     * Runs the PID velocity control loop when the flywheel is running.
     * Also performs fault detection.
     */
    public void update() {
        double currentSpeed = motorUtils.getMotorVelocityTicksPerSec(flyWheelMotor);
        long now = System.currentTimeMillis();

        if (running) {
            double power = pid.calculate(targetSpeed, currentSpeed);
            flyWheelMotor.setPower(power);

            // Fault detection
            checkFaults(now, currentSpeed, power);
        } else {
            flyWheelMotor.setPower(0);
        }

        // Report telemetry
        reportTelemetry(currentSpeed);
    }

    /**
     * Checks for motor stall, encoder fault, PID saturation, and speed error.
     */
    private void checkFaults(long nowMilliseconds, double currentSpeed, double pidOutput) {
        int currentPosition = flyWheelMotor.getCurrentPosition();
        double motorPower = flyWheelMotor.getPower();

        // Encoder fault: position unchanged while powered
        if (currentPosition != lastPosition) {
            lastPosition = currentPosition;
            lastPositionChangeMilliseconds = nowMilliseconds;
            encoderFault = false;
        } else {
            if (Math.abs(motorPower) > RobotConstants.FLYWHEEL_ENCODER_FAULT_POWER_THRESHOLD
                    && nowMilliseconds - lastPositionChangeMilliseconds
                            > RobotConstants.FLYWHEEL_ENCODER_FAULT_TIMEOUT_MS) {
                encoderFault = true;
            }
        }

        // Motor stall: powered but not moving
        if (Math.abs(motorPower) > RobotConstants.FLYWHEEL_STALL_POWER_THRESHOLD
                && Math.abs(currentSpeed) < RobotConstants.FLYWHEEL_STALL_SPEED_THRESHOLD) {
            if (nowMilliseconds - lastPositionChangeMilliseconds
                    > (long) (RobotConstants.MOTOR_STALL_TIMEOUT_SECONDS * 1000)) {
                stallDetected = true;
            }
        } else {
            stallDetected = false;
        }

        // PID saturation
        pidSaturated = pid.isSaturated();

        // Speed error exceeded threshold
        double speedError = Math.abs(targetSpeed - currentSpeed);
        speedErrorExceeded = speedError > (targetSpeed * RobotConstants.FLYWHEEL_SPEED_ERROR_THRESHOLD_FRACTION);
    }

    /**
     * Reports telemetry to TelemetryServer.
     */
    private void reportTelemetry(double currentSpeed) {
        TelemetryServer.getInstance().setSubsystemTelemetry("flyWheel", "targetSpeed", targetSpeed);
        TelemetryServer.getInstance().setSubsystemTelemetry("flyWheel", "currentSpeed", currentSpeed);
        TelemetryServer.getInstance().setSubsystemTelemetry("flyWheel", "running", running);
        TelemetryServer.getInstance().setSubsystemTelemetry("flyWheel", "motorPower", flyWheelMotor.getPower());
        TelemetryServer.getInstance().setSubsystemTelemetry("flyWheel", "stallDetected", stallDetected);
        TelemetryServer.getInstance().setSubsystemTelemetry("flyWheel", "encoderFault", encoderFault);
        TelemetryServer.getInstance().setSubsystemTelemetry("flyWheel", "pidSaturated", pidSaturated);
        TelemetryServer.getInstance().setSubsystemTelemetry("flyWheel", "speedErrorExceeded", speedErrorExceeded);
        TelemetryServer.getInstance().setPidGraph(targetSpeed, currentSpeed);
    }

    /** Returns the current flywheel speed in encoder ticks per second. */
    public double getCurrentSpeed() {
        return motorUtils.getMotorVelocityTicksPerSec(flyWheelMotor);
    }

    /** Returns the current target speed in encoder ticks per second. */
    public double getTargetSpeed() {
        return targetSpeed;
    }

    /** Returns whether the flywheel is currently running. */
    public boolean isRunning() {
        return running;
    }

    /** Returns whether a motor stall has been detected. */
    public boolean isStallDetected() {
        return stallDetected;
    }

    /** Returns whether an encoder fault has been detected. */
    public boolean isEncoderFault() {
        return encoderFault;
    }

    /** Returns whether the PID output is saturated. */
    public boolean isPidSaturated() {
        return pidSaturated;
    }

    /** Returns whether the speed error exceeds the threshold. */
    public boolean isSpeedErrorExceeded() {
        return speedErrorExceeded;
    }
}
