package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;
import org.firstinspires.ftc.teamcode.util.EmergencyStop;
import org.firstinspires.ftc.teamcode.util.MotorErrorTracker;
import org.firstinspires.ftc.teamcode.util.MotorFaultDetector;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.util.PIDController;

/**
 * FlyWheel subsystem with PID velocity control and centralized motor fault detection.
 */
public class FlyWheel {

    private final DcMotor flyWheelMotor;
    private final MotorFaultDetector faultDetector;
    private final PIDController pid = new PIDController(
            RobotConstants.FLYWHEEL_P,
            RobotConstants.FLYWHEEL_I,
            RobotConstants.FLYWHEEL_D,
            RobotConstants.FLYWHEEL_F
    );

    private volatile double targetSpeed = RobotConstants.FLYWHEEL_DEFAULT_TARGET_SPEED;
    private boolean running = false;

    private boolean pidSaturated = false;
    private boolean speedErrorExceeded = false;
    private double lastCurrentSpeed = 0.0;
    private double lastPower = 0.0;

    public static final String SUBSYSTEM_KEY = "flyWheel";

    public FlyWheel(HardwareMap hardwareMap) {
        flyWheelMotor = hardwareMap.get(DcMotor.class, HardwareNames.FLY_WHEEL_MOTOR);
        MotorUtils.clearMotorState(flyWheelMotor);

        faultDetector = new MotorFaultDetector(flyWheelMotor, "flyWheel", "flywheel");
        faultDetector.setPresetThresholds("flywheel");
        MotorErrorTracker.getInstance().register(faultDetector);

        TelemetryServer.getInstance().setTargetedPidListener((target, proportional, integral, derivative, feedforward, setpoint) -> {
            if ("SUBSYSTEM_KEY".equals(target)) {
                pid.setPIDF(proportional, integral, derivative, feedforward);
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
        if (EmergencyStop.getInstance().isSubsystemStopped(SUBSYSTEM_KEY)) {
            return;
        }
        running = true;
        this.targetSpeed = targetSpeed;
        pid.reset();
    }

    /** Starts the flywheel at the default target speed from RobotConstants. */
    public void start() {
        start(RobotConstants.FLYWHEEL_DEFAULT_TARGET_SPEED);
    }

    /** Stops the flywheel. */
    public void stop() {
        running = false;
        lastPower = 0.0;
        flyWheelMotor.setPower(0.0);
    }

    /**
     * Toggles the flywheel on/off.
     * If an Auto E-Stop is active on the flywheel, the first press clears the
     * E-Stop latch (keeping the flywheel off) so the driver can safely restart.
     */
    public void toggle() {
        EmergencyStop eStop = EmergencyStop.getInstance();
        if (eStop.isGlobalEStopped()) {
            return;
        }
        if (eStop.isSubsystemAutoEStopped(SUBSYSTEM_KEY)) {
            clearAutoEStop();
            return;
        }
        running = !running;
        if (running) {
            pid.reset();
        }
    }

    /** Clears an active Auto E-Stop on the flywheel and resets its fault detector. */
    public void clearAutoEStop() {
        EmergencyStop.getInstance().clearSubsystem(SUBSYSTEM_KEY);
        faultDetector.reset();
        running = false;
        pidSaturated = false;
        speedErrorExceeded = false;
        pid.reset();
    }

    /** Returns true if the flywheel is currently locked by global or auto E-Stop. */
    public boolean isEStopped() {
        return EmergencyStop.getInstance().isSubsystemStopped(SUBSYSTEM_KEY);
    }

    /**
     * Call this once per loop from the OpMode.
     */
    public void update() {
        double currentSpeed = MotorUtils.getMotorVelocityTicksPerSec(flyWheelMotor);
        lastCurrentSpeed = currentSpeed;
        double currentTarget = targetSpeed;

        EmergencyStop eStop = EmergencyStop.getInstance();
        if (eStop.isSubsystemStopped(SUBSYSTEM_KEY)) {
            running = false;
            lastPower = 0.0;
            flyWheelMotor.setPower(0.0);
            pidSaturated = false;
            speedErrorExceeded = false;
            pid.reset();
            faultDetector.update(0.0, flyWheelMotor.getCurrentPosition(), currentSpeed);
            faultDetector.updatePidStatus(false, false, currentTarget, currentSpeed, 0.0);
            reportTelemetry(currentSpeed, currentTarget);
            return;
        }

        if (running) {
            double power = pid.calculate(currentTarget, currentSpeed);
            lastPower = power;
            flyWheelMotor.setPower(power);

            faultDetector.update(power, flyWheelMotor.getCurrentPosition(), currentSpeed);

            pidSaturated = pid.isSaturated();
            double allowedError = Math.abs(currentTarget) * RobotConstants.FLYWHEEL_SPEED_ERROR_THRESHOLD_FRACTION;
            double speedError = Math.abs(currentTarget - currentSpeed);
            speedErrorExceeded = speedError > allowedError;

            faultDetector.updatePidStatus(true, pidSaturated, currentTarget, currentSpeed, allowedError);

            if (shouldTriggerAutoEStop()) {
                String reason = faultDetector.getPrimaryError().getShortLabel()
                        + ": " + faultDetector.getErrorMessage();
                eStop.triggerSubsystem(SUBSYSTEM_KEY, reason);
                running = false;
                lastPower = 0.0;
                flyWheelMotor.setPower(0.0);
                pidSaturated = false;
                speedErrorExceeded = false;
                pid.reset();
            }
        } else {
            lastPower = 0.0;
            flyWheelMotor.setPower(0.0);
            pidSaturated = false;
            speedErrorExceeded = false;
            faultDetector.update(0.0, flyWheelMotor.getCurrentPosition(), currentSpeed);
            faultDetector.updatePidStatus(false, false, currentTarget, currentSpeed, 0.0);
        }

        reportTelemetry(currentSpeed, currentTarget);
    }

    private boolean shouldTriggerAutoEStop() {
        if (!RobotConstants.AUTO_ESTOP_FLYWHEEL_ENABLED) {
            return false;
        }
        return (RobotConstants.AUTO_ESTOP_FLYWHEEL_ON_STALL && faultDetector.isStallDetected())
                || (RobotConstants.AUTO_ESTOP_FLYWHEEL_ON_ENCODER_FAULT && faultDetector.isEncoderFault())
                || (RobotConstants.AUTO_ESTOP_FLYWHEEL_ON_OVERCURRENT && faultDetector.isOvercurrentDetected());
    }

    private void reportTelemetry(double currentSpeed, double currentTarget) {
        TelemetryServer server = TelemetryServer.getInstance();
        server.setSubsystemTelemetry("flyWheel", "targetSpeed", currentTarget);
        server.setSubsystemTelemetry("flyWheel", "currentSpeed", currentSpeed);
        server.setSubsystemTelemetry("flyWheel", "running", running);
        server.setSubsystemTelemetry("flyWheel", "motorPower", lastPower);
        server.setSubsystemTelemetry("flyWheel", "stallDetected", isStallDetected());
        server.setSubsystemTelemetry("flyWheel", "encoderFault", isEncoderFault());
        server.setSubsystemTelemetry("flyWheel", "pidSaturated", pidSaturated);
        server.setSubsystemTelemetry("flyWheel", "speedErrorExceeded", speedErrorExceeded);
        server.setSubsystemTelemetry("flyWheel", "error", faultDetector.getPrimaryError().name());
        server.setSubsystemTelemetry("flyWheel", "eStopped", isEStopped());
        server.setPidGraph(currentTarget, currentSpeed);
    }

    public double getCurrentSpeed() {
        return lastCurrentSpeed;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public double getMotorPower() {
        return lastPower;
    }

    public boolean isRunning() {
        return running;
    }

    public boolean isStallDetected() {
        return faultDetector.isStallDetected();
    }

    public boolean isEncoderFault() {
        return faultDetector.isEncoderFault();
    }

    public boolean isPidSaturated() {
        return pidSaturated;
    }

    public boolean isSpeedErrorExceeded() {
        return speedErrorExceeded;
    }
}
