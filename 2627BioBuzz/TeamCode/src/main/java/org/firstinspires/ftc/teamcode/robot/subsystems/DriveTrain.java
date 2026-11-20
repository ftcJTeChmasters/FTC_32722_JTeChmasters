package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * DriveTrain subsystem with acceleration ramping (ease in / ease out).
 *
 * setPowers()/setLeftPower()/setRightPower() set TARGET powers.
 * update() MUST be called once per loop; it ramps the real motor power
 * toward the target so the wheels don't slip.
 *
 * When power reaches 0 the motors BRAKE (resist rotation) instead of coasting.
 */
public class DriveTrain {
    // Max change in power per second. 1.0 power/sec = 0 -> full in 1 s.
    // Tune these on your carpet/field tiles.
    private double accelRate = 5.0;   // speeding up (ease in)   ~0.33 s to full power
    private double decelRate = 25.0;   // slowing down (ease out) ~0.20 s to stop

    private static final double MAX_DT = 0.1; // ignore loop hiccups longer than this

    private final DcMotor leftDriveMotor;
    private final DcMotor rightDriveMotor;
    private final MotorUtils motorUtils = new MotorUtils();

    public double speedMultiplier = 1; // scale down power for precision driving

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    private double targetLeft = 0, targetRight = 0;
    private double currentLeft = 0, currentRight = 0;

    public DriveTrain(HardwareMap hardwareMap) {
        leftDriveMotor = hardwareMap.get(DcMotor.class, HardwareNames.LEFT_DRIVE_MOTOR);
        rightDriveMotor = hardwareMap.get(DcMotor.class, HardwareNames.RIGHT_DRIVE_MOTOR);
        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        // Brake instead of coast whenever power is 0
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer.reset();
    }

    /** Tune ramp speeds (power change per second). */
    public void setRampRates(double accelPerSec, double decelPerSec) {
        this.accelRate = Math.max(0.01, accelPerSec);
        this.decelRate = Math.max(0.01, decelPerSec);
    }

    // ---- Targets (eased) ----

    public void setLeftPower(double power) {
        targetLeft = clip(power * speedMultiplier);
    }

    public void setRightPower(double power) {
        targetRight = clip(power * speedMultiplier);
    }

    public void setPowers(double leftPower, double rightPower) {
        targetLeft = clip(leftPower * speedMultiplier);
        targetRight = clip(rightPower * speedMultiplier);
    }

    /** Eased stop: ramps down to 0 using decelRate, then brakes. */
    public void stop() {
        targetLeft = 0;
        targetRight = 0;
    }

    /** Immediate stop, bypasses easing. Use for emergencies / end of OpMode. */
    public void stopImmediately() {
        targetLeft = targetRight = 0;
        currentLeft = currentRight = 0;
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

    // ---- Continuous control ----

    /** Call this once per loop from the OpMode. */
    public void update() {
        double now = timer.seconds();
        double dt = Math.min(now - lastTime, MAX_DT);
        lastTime = now;

        currentLeft = ramp(currentLeft, targetLeft, dt);
        currentRight = ramp(currentRight, targetRight, dt);

        leftDriveMotor.setPower(currentLeft);
        rightDriveMotor.setPower(currentRight);
    }

    private double ramp(double current, double target, double dt) {
        // Decelerating = target is closer to zero than current, on the same side.
        // (Reversing direction also counts: we first brake toward 0, then accelerate.)
        boolean decelerating = Math.abs(target) < Math.abs(current)
                || Math.signum(target) != Math.signum(current) && current != 0;

        double maxStep = (decelerating ? decelRate : accelRate) * dt;
        double error = target - current;

        if (Math.abs(error) <= maxStep) return target;
        return current + Math.copySign(maxStep, error);
    }

    private double clip(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }
}