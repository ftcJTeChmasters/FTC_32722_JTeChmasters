package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;

/**
 * DriveTrain subsystem with acceleration ramping (ease in / ease out).
 * <p>
 * {@link #setPowers(double, double)}, {@link #setLeftPower(double)},
 * and {@link #setRightPower(double)} set TARGET powers.
 * {@link #update()} MUST be called once per loop; it ramps the real motor
 * power toward the target so the wheels don't slip.
 * <p>
 * When power reaches 0 the motors BRAKE (resist rotation) instead of coasting.
 */
public class DriveTrain {
    // Max change in power per second. 1.0 power/sec = 0 -> full in 1 s.
    // Tune these on your carpet/field tiles.
    private double accelerationRate = RobotConstants.DRIVETRAIN_ACCELERATION_RATE;
    private double decelerationRate = RobotConstants.DRIVETRAIN_DECELERATION_RATE;

    private static final double MAX_DELTA_TIME =
            RobotConstants.DRIVETRAIN_MAX_DELTA_TIME_SECONDS; // ignore loop hiccups longer than this

    private final DcMotor leftDriveMotor;
    private final DcMotor rightDriveMotor;
    private final MotorUtils motorUtils = new MotorUtils();

    /** Scale factor for precision driving (0.0 to 1.0). */
    public double speedMultiplier = 1;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTimeSeconds = 0;

    private double targetLeftPower = 0;
    private double targetRightPower = 0;
    private double currentLeftPower = 0;
    private double currentRightPower = 0;

    // Fault detection
    private long lastLeftPositionChangeMilliseconds = 0;
    private long lastRightPositionChangeMilliseconds = 0;
    private int lastLeftPosition = 0;
    private int lastRightPosition = 0;
    private boolean leftStallDetected = false;
    private boolean rightStallDetected = false;
    private boolean leftEncoderFault = false;
    private boolean rightEncoderFault = false;

    public DriveTrain(HardwareMap hardwareMap) {
        leftDriveMotor = hardwareMap.get(DcMotor.class, HardwareNames.LEFT_DRIVE_MOTOR);
        rightDriveMotor = hardwareMap.get(DcMotor.class, HardwareNames.RIGHT_DRIVE_MOTOR);
        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        // Brake instead of coast whenever power is 0
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Clear any stale velocity state from previous OpMode runs
        MotorUtils.clearMotorState(leftDriveMotor);
        MotorUtils.clearMotorState(rightDriveMotor);

        // Initialize fault detection
        lastLeftPosition = leftDriveMotor.getCurrentPosition();
        lastRightPosition = rightDriveMotor.getCurrentPosition();
        lastLeftPositionChangeMilliseconds = System.currentTimeMillis();
        lastRightPositionChangeMilliseconds = System.currentTimeMillis();

        timer.reset();
    }

    /** Tune ramp speeds (power change per second). */
    public void setRampRates(double accelerationPerSec, double decelerationPerSec) {
        this.accelerationRate = Math.max(0.01, accelerationPerSec);
        this.decelerationRate = Math.max(0.01, decelerationPerSec);
    }

    // ---- Targets (eased) ----

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = Math.max(0, multiplier);
    }

    public void setLeftPower(double power) {
        targetLeftPower = clip(power * speedMultiplier);
    }

    public void setRightPower(double power) {
        targetRightPower = clip(power * speedMultiplier);
    }

    public void setPowers(double leftPower, double rightPower) {
        targetLeftPower = clip(leftPower * speedMultiplier);
        targetRightPower = clip(rightPower * speedMultiplier);
    }

    /** Eased stop: ramps down to 0 using decelRate, then brakes. */
    public void stop() {
        targetLeftPower = 0;
        targetRightPower = 0;
    }

    /** Immediate stop, bypasses easing. Use for emergencies / end of OpMode. */
    public void stopImmediately() {
        targetLeftPower = targetRightPower = 0;
        currentLeftPower = currentRightPower = 0;
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
    }

    // ---- Sensors ----

    public double getLeftSpeed() {
        return motorUtils.getMotorVelocityTicksPerSec(leftDriveMotor);
    }

    public double getRightSpeed() {
        return motorUtils.getMotorVelocityTicksPerSec(rightDriveMotor);
    }

    public void getSpeeds(double[] speeds) {
        speeds[0] = getLeftSpeed();
        speeds[1] = getRightSpeed();
    }

    /**
     * Returns whether the left motor has a stall detected.
     */
    public boolean isLeftStallDetected() {
        return leftStallDetected;
    }

    /**
     * Returns whether the right motor has a stall detected.
     */
    public boolean isRightStallDetected() {
        return rightStallDetected;
    }

    /**
     * Returns whether the left motor has an encoder fault.
     */
    public boolean isLeftEncoderFault() {
        return leftEncoderFault;
    }

    /**
     * Returns whether the right motor has an encoder fault.
     */
    public boolean isRightEncoderFault() {
        return rightEncoderFault;
    }

    // ---- Continuous control ----

    /** Call this once per loop from the OpMode. */
    public void update() {
        double now = timer.seconds();
        double dt = Math.min(now - lastTimeSeconds, MAX_DELTA_TIME);
        lastTimeSeconds = now;

        currentLeftPower = ramp(currentLeftPower, targetLeftPower, dt);
        currentRightPower = ramp(currentRightPower, targetRightPower, dt);

        leftDriveMotor.setPower(currentLeftPower);
        rightDriveMotor.setPower(currentRightPower);

        // Fault detection
        checkFaults(System.currentTimeMillis());

        // Report telemetry
        reportTelemetry();
    }

    private double ramp(double current, double target, double dt) {
        // Decelerating = target is closer to zero than current, on the same side.
        // (Reversing direction also counts: we first brake toward 0, then accelerate.)
        boolean decelerating = Math.abs(target) < Math.abs(current)
                || Math.signum(target) != Math.signum(current) && current != 0;

        double maxStep = (decelerating ? decelerationRate : accelerationRate) * dt;
        double error = target - current;

        if (Math.abs(error) <= maxStep) return target;
        return current + Math.copySign(maxStep, error);
    }

    private double clip(double velocity) {
        return Math.max(-1.0, Math.min(1.0, velocity));
    }

    /**
     * Checks for encoder faults and motor stalls on both drive motors.
     */
    private void checkFaults(long nowMilliseconds) {
        // Left motor
        int leftPos = leftDriveMotor.getCurrentPosition();
        if (leftPos != lastLeftPosition) {
            lastLeftPosition = leftPos;
            lastLeftPositionChangeMilliseconds = nowMilliseconds;
            leftEncoderFault = false;
        } else {
            double leftPower = leftDriveMotor.getPower();
            if (Math.abs(leftPower) > RobotConstants.DRIVETRAIN_ENCODER_FAULT_POWER_THRESHOLD
                    && nowMilliseconds - lastLeftPositionChangeMilliseconds
                            > RobotConstants.DRIVETRAIN_ENCODER_FAULT_TIMEOUT_MS) {
                leftEncoderFault = true;
            }
        }

        // Right motor
        int rightPos = rightDriveMotor.getCurrentPosition();
        if (rightPos != lastRightPosition) {
            lastRightPosition = rightPos;
            lastRightPositionChangeMilliseconds = nowMilliseconds;
            rightEncoderFault = false;
        } else {
            double rightPower = rightDriveMotor.getPower();
            if (Math.abs(rightPower) > RobotConstants.DRIVETRAIN_ENCODER_FAULT_POWER_THRESHOLD
                    && nowMilliseconds - lastRightPositionChangeMilliseconds
                            > RobotConstants.DRIVETRAIN_ENCODER_FAULT_TIMEOUT_MS) {
                rightEncoderFault = true;
            }
        }

        // Stall detection
        double leftSpeed = getLeftSpeed();
        double rightSpeed = getRightSpeed();
        double leftPower = leftDriveMotor.getPower();
        double rightPower = rightDriveMotor.getPower();

        leftStallDetected = Math.abs(leftPower) > RobotConstants.DRIVETRAIN_STALL_POWER_THRESHOLD
                && Math.abs(leftSpeed) < RobotConstants.DRIVETRAIN_STALL_SPEED_THRESHOLD;
        rightStallDetected = Math.abs(rightPower) > RobotConstants.DRIVETRAIN_STALL_POWER_THRESHOLD
                && Math.abs(rightSpeed) < RobotConstants.DRIVETRAIN_STALL_SPEED_THRESHOLD;
    }

    /**
     * Reports telemetry to TelemetryServer.
     */
    private void reportTelemetry() {
        TelemetryServer.getInstance().setSubsystemTelemetry("driveTrain", "leftPower", currentLeftPower);
        TelemetryServer.getInstance().setSubsystemTelemetry("driveTrain", "rightPower", currentRightPower);
        TelemetryServer.getInstance().setSubsystemTelemetry("driveTrain", "leftSpeed", getLeftSpeed());
        TelemetryServer.getInstance().setSubsystemTelemetry("driveTrain", "rightSpeed", getRightSpeed());
        TelemetryServer.getInstance().setSubsystemTelemetry("driveTrain", "speedMultiplier", speedMultiplier);
        TelemetryServer.getInstance().setSubsystemTelemetry("driveTrain", "leftStall", leftStallDetected);
        TelemetryServer.getInstance().setSubsystemTelemetry("driveTrain", "rightStall", rightStallDetected);
        TelemetryServer.getInstance().setSubsystemTelemetry("driveTrain", "leftEncoderFault", leftEncoderFault);
        TelemetryServer.getInstance().setSubsystemTelemetry("driveTrain", "rightEncoderFault", rightEncoderFault);
    }
}