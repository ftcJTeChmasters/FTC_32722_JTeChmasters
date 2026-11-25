package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;
import org.firstinspires.ftc.teamcode.robot.odometry.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.util.EmergencyStop;
import org.firstinspires.ftc.teamcode.util.MotorErrorTracker;
import org.firstinspires.ftc.teamcode.util.MotorFaultDetector;
import org.firstinspires.ftc.teamcode.util.MotorUtils;

/**
 * DriveTrain subsystem supporting both:
 * <ul>
 *   <li>Legacy 2-motor tank drive ({@link RobotConstants#MECANUM_ODO_INSTALLED} == {@code false})</li>
 *   <li>4-motor mecanum drive + 2-pod IMU odometry ({@link RobotConstants#MECANUM_ODO_INSTALLED} == {@code true})</li>
 * </ul>
 * <p>
 * Provides smooth acceleration/deceleration ramping, BRAKE zero-power behavior,
 * and centralized motor error monitoring via {@link MotorFaultDetector} and
 * {@link MotorErrorTracker}.
 */
public class DriveTrain {
    private double accelerationRate = RobotConstants.DRIVETRAIN_ACCELERATION_RATE;
    private double decelerationRate = RobotConstants.DRIVETRAIN_DECELERATION_RATE;

    private static final double MAX_DELTA_TIME = RobotConstants.DRIVETRAIN_MAX_DELTA_TIME_SECONDS;

    private final boolean mecanumInstalled = RobotConstants.MECANUM_ODO_INSTALLED;

    // Legacy 2-motor tank hardware
    private final DcMotor leftDriveMotor;
    private final DcMotor rightDriveMotor;
    private final MotorFaultDetector leftFaultDetector;
    private final MotorFaultDetector rightFaultDetector;

    // New 4-motor mecanum + odometry hardware
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;
    private final MotorFaultDetector frontLeftFaultDetector;
    private final MotorFaultDetector frontRightFaultDetector;
    private final MotorFaultDetector backLeftFaultDetector;
    private final MotorFaultDetector backRightFaultDetector;
    private final TwoWheelOdometry odometry;

    /** Scale factor for precision driving (0.0 to 1.0). */
    public double speedMultiplier = 1.0;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTimeSeconds = 0.0;

    // Legacy tank powers
    private double targetLeftPower = 0.0;
    private double targetRightPower = 0.0;
    private double currentLeftPower = 0.0;
    private double currentRightPower = 0.0;
    private double lastLeftSpeed = 0.0;
    private double lastRightSpeed = 0.0;

    // Mecanum powers
    private double targetFrontLeftPower = 0.0;
    private double targetFrontRightPower = 0.0;
    private double targetBackLeftPower = 0.0;
    private double targetBackRightPower = 0.0;
    private double currentFrontLeftPower = 0.0;
    private double currentFrontRightPower = 0.0;
    private double currentBackLeftPower = 0.0;
    private double currentBackRightPower = 0.0;

    public DriveTrain(HardwareMap hardwareMap) {
        if (mecanumInstalled) {
            leftDriveMotor = null;
            rightDriveMotor = null;
            leftFaultDetector = null;
            rightFaultDetector = null;

            frontLeftMotor = configureMecanumMotor(
                    hardwareMap,
                    HardwareNames.FRONT_LEFT_DRIVE_MOTOR,
                    RobotConstants.MECANUM_REVERSE_FRONT_LEFT
            );
            frontRightMotor = configureMecanumMotor(
                    hardwareMap,
                    HardwareNames.FRONT_RIGHT_DRIVE_MOTOR,
                    RobotConstants.MECANUM_REVERSE_FRONT_RIGHT
            );
            backLeftMotor = configureMecanumMotor(
                    hardwareMap,
                    HardwareNames.BACK_LEFT_DRIVE_MOTOR,
                    RobotConstants.MECANUM_REVERSE_BACK_LEFT
            );
            backRightMotor = configureMecanumMotor(
                    hardwareMap,
                    HardwareNames.BACK_RIGHT_DRIVE_MOTOR,
                    RobotConstants.MECANUM_REVERSE_BACK_RIGHT
            );

            frontLeftFaultDetector = createDriveFaultDetector(frontLeftMotor, "frontLeft");
            frontRightFaultDetector = createDriveFaultDetector(frontRightMotor, "frontRight");
            backLeftFaultDetector = createDriveFaultDetector(backLeftMotor, "backLeft");
            backRightFaultDetector = createDriveFaultDetector(backRightMotor, "backRight");

            odometry = new TwoWheelOdometry(hardwareMap);
        } else {
            frontLeftMotor = null;
            frontRightMotor = null;
            backLeftMotor = null;
            backRightMotor = null;
            frontLeftFaultDetector = null;
            frontRightFaultDetector = null;
            backLeftFaultDetector = null;
            backRightFaultDetector = null;
            odometry = null;

            leftDriveMotor = hardwareMap.get(DcMotor.class, HardwareNames.LEFT_DRIVE_MOTOR);
            rightDriveMotor = hardwareMap.get(DcMotor.class, HardwareNames.RIGHT_DRIVE_MOTOR);
            leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            MotorUtils.clearMotorState(leftDriveMotor);
            MotorUtils.clearMotorState(rightDriveMotor);

            leftFaultDetector = createDriveFaultDetector(leftDriveMotor, "leftDrive");
            rightFaultDetector = createDriveFaultDetector(rightDriveMotor, "rightDrive");
            leftFaultDetector.setEncoderCheckEnabled(true);
            rightFaultDetector.setEncoderCheckEnabled(true);
        }

        timer.reset();
    }

    private DcMotor configureMecanumMotor(HardwareMap hardwareMap, String name, boolean reverse) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorUtils.clearMotorState(motor);
        return motor;
    }

    private MotorFaultDetector createDriveFaultDetector(DcMotor motor, String motorName) {
        MotorFaultDetector detector = new MotorFaultDetector(motor, "driveTrain", motorName);
        detector.setPresetThresholds("drivetrain");
        detector.setEncoderCheckEnabled(!mecanumInstalled || RobotConstants.MECANUM_MOTORS_HAVE_ENCODERS);
        MotorErrorTracker.getInstance().register(detector);
        return detector;
    }

    /** Returns true if the new 4-motor mecanum + odometry drivetrain is enabled. */
    public boolean isMecanumInstalled() {
        return mecanumInstalled;
    }

    /** Tune ramp speeds (power change per second). */
    public void setRampRates(double accelerationPerSec, double decelerationPerSec) {
        this.accelerationRate = Math.max(RobotConstants.DRIVETRAIN_MIN_RAMP_RATE, accelerationPerSec);
        this.decelerationRate = Math.max(RobotConstants.DRIVETRAIN_MIN_RAMP_RATE, decelerationPerSec);
    }

    // ---- Targets (eased) ----

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = Math.max(0.0, multiplier);
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void setLeftPower(double power) {
        double scaled = clip(power * speedMultiplier);
        if (mecanumInstalled) {
            targetFrontLeftPower = scaled;
            targetBackLeftPower = scaled;
        } else {
            targetLeftPower = scaled;
        }
    }

    public void setRightPower(double power) {
        double scaled = clip(power * speedMultiplier);
        if (mecanumInstalled) {
            targetFrontRightPower = scaled;
            targetBackRightPower = scaled;
        } else {
            targetRightPower = scaled;
        }
    }

    /**
     * Sets tank-style left and right target powers (works in both legacy tank
     * and mecanum modes so autonomous routines like AutoLeave stay compatible).
     */
    public void setPowers(double leftPower, double rightPower) {
        double scaledLeft = clip(leftPower * speedMultiplier);
        double scaledRight = clip(rightPower * speedMultiplier);
        if (mecanumInstalled) {
            targetFrontLeftPower = scaledLeft;
            targetBackLeftPower = scaledLeft;
            targetFrontRightPower = scaledRight;
            targetBackRightPower = scaledRight;
        } else {
            targetLeftPower = scaledLeft;
            targetRightPower = scaledRight;
        }
    }

    /**
     * Sets robot-centric mecanum target powers.
     * <p>
     * When running in legacy 2-motor mode, falls back to arcade drive
     * ({@code forward + turn} / {@code forward - turn}).
     *
     * @param forward forward velocity (-1.0 to 1.0, positive = forward)
     * @param strafe  strafe velocity (-1.0 to 1.0, positive = right)
     * @param turn    rotation velocity (-1.0 to 1.0, positive = clockwise/right)
     */
    public void setMecanumPowers(double forward, double strafe, double turn) {
        if (!mecanumInstalled) {
            double left = forward + turn;
            double right = forward - turn;
            double maxMag = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
            setPowers(left / maxMag, right / maxMag);
            return;
        }

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double maxMagnitude = Math.max(
                1.0,
                Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)))
        );

        targetFrontLeftPower = clip((fl / maxMagnitude) * speedMultiplier);
        targetFrontRightPower = clip((fr / maxMagnitude) * speedMultiplier);
        targetBackLeftPower = clip((bl / maxMagnitude) * speedMultiplier);
        targetBackRightPower = clip((br / maxMagnitude) * speedMultiplier);
    }

    /** Eased stop: ramps down to 0 using decelRate, then brakes. */
    public void stop() {
        targetLeftPower = 0.0;
        targetRightPower = 0.0;
        targetFrontLeftPower = 0.0;
        targetFrontRightPower = 0.0;
        targetBackLeftPower = 0.0;
        targetBackRightPower = 0.0;
    }

    /** Immediate stop, bypasses easing. Use for emergencies / end of OpMode. */
    public void stopImmediately() {
        stop();
        currentLeftPower = 0.0;
        currentRightPower = 0.0;
        currentFrontLeftPower = 0.0;
        currentFrontRightPower = 0.0;
        currentBackLeftPower = 0.0;
        currentBackRightPower = 0.0;
        if (mecanumInstalled) {
            frontLeftMotor.setPower(0.0);
            frontRightMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);
            backRightMotor.setPower(0.0);
        } else {
            leftDriveMotor.setPower(0.0);
            rightDriveMotor.setPower(0.0);
        }
    }

    // ---- Sensors & Pose ----

    public TwoWheelOdometry getOdometry() {
        return odometry;
    }

    public void resetOdometry() {
        if (odometry != null) {
            odometry.resetPose();
        }
    }

    public double getPoseX() {
        return odometry != null ? odometry.getXInches() : 0.0;
    }

    public double getPoseY() {
        return odometry != null ? odometry.getYInches() : 0.0;
    }

    public double getHeadingDegrees() {
        return odometry != null ? odometry.getHeadingDegrees() : 0.0;
    }

    public double getLeftPower() {
        return mecanumInstalled ? 0.5 * (currentFrontLeftPower + currentBackLeftPower) : currentLeftPower;
    }

    public double getRightPower() {
        return mecanumInstalled ? 0.5 * (currentFrontRightPower + currentBackRightPower) : currentRightPower;
    }

    public double getFrontLeftPower() {
        return currentFrontLeftPower;
    }

    public double getFrontRightPower() {
        return currentFrontRightPower;
    }

    public double getBackLeftPower() {
        return currentBackLeftPower;
    }

    public double getBackRightPower() {
        return currentBackRightPower;
    }

    public double getLeftSpeed() {
        if (mecanumInstalled) {
            return 0.5 * (MotorUtils.getMotorVelocityTicksPerSec(frontLeftMotor)
                    + MotorUtils.getMotorVelocityTicksPerSec(backLeftMotor));
        }
        return MotorUtils.getMotorVelocityTicksPerSec(leftDriveMotor);
    }

    public double getRightSpeed() {
        if (mecanumInstalled) {
            return 0.5 * (MotorUtils.getMotorVelocityTicksPerSec(frontRightMotor)
                    + MotorUtils.getMotorVelocityTicksPerSec(backRightMotor));
        }
        return MotorUtils.getMotorVelocityTicksPerSec(rightDriveMotor);
    }

    public void getSpeeds(double[] speeds) {
        speeds[0] = getLeftSpeed();
        speeds[1] = getRightSpeed();
    }

    public double getLastLeftSpeed() {
        return lastLeftSpeed;
    }

    public double getLastRightSpeed() {
        return lastRightSpeed;
    }

    public boolean isLeftStallDetected() {
        if (mecanumInstalled) {
            return frontLeftFaultDetector.isStallDetected() || backLeftFaultDetector.isStallDetected();
        }
        return leftFaultDetector.isStallDetected();
    }

    public boolean isRightStallDetected() {
        if (mecanumInstalled) {
            return frontRightFaultDetector.isStallDetected() || backRightFaultDetector.isStallDetected();
        }
        return rightFaultDetector.isStallDetected();
    }

    public boolean isLeftEncoderFault() {
        if (mecanumInstalled) {
            return frontLeftFaultDetector.isEncoderFault() || backLeftFaultDetector.isEncoderFault();
        }
        return leftFaultDetector.isEncoderFault();
    }

    public boolean isRightEncoderFault() {
        if (mecanumInstalled) {
            return frontRightFaultDetector.isEncoderFault() || backRightFaultDetector.isEncoderFault();
        }
        return rightFaultDetector.isEncoderFault();
    }

    public static final String SUBSYSTEM_KEY = "driveTrain";

    /** Returns true if the drivetrain is currently locked by global or auto E-Stop. */
    public boolean isEStopped() {
        return EmergencyStop.getInstance().isSubsystemStopped(SUBSYSTEM_KEY);
    }

    /** Clears an active Auto E-Stop on the drivetrain and resets drive fault detectors. */
    public void clearAutoEStop() {
        EmergencyStop.getInstance().clearSubsystem(SUBSYSTEM_KEY);
        if (mecanumInstalled) {
            frontLeftFaultDetector.reset();
            frontRightFaultDetector.reset();
            backLeftFaultDetector.reset();
            backRightFaultDetector.reset();
        } else {
            leftFaultDetector.reset();
            rightFaultDetector.reset();
        }
    }

    // ---- Continuous control ----

    /** Call this once per loop from the OpMode. */
    public void update() {
        double now = timer.seconds();
        double dt = Math.min(Math.max(0.0, now - lastTimeSeconds), MAX_DELTA_TIME);
        lastTimeSeconds = now;

        EmergencyStop eStop = EmergencyStop.getInstance();
        if (eStop.isSubsystemStopped(SUBSYSTEM_KEY)) {
            stopImmediately();
            if (mecanumInstalled && odometry != null) {
                odometry.update();
            }
            reportTelemetry();
            return;
        }

        if (mecanumInstalled) {
            currentFrontLeftPower = ramp(currentFrontLeftPower, targetFrontLeftPower, dt);
            currentFrontRightPower = ramp(currentFrontRightPower, targetFrontRightPower, dt);
            currentBackLeftPower = ramp(currentBackLeftPower, targetBackLeftPower, dt);
            currentBackRightPower = ramp(currentBackRightPower, targetBackRightPower, dt);

            frontLeftMotor.setPower(currentFrontLeftPower);
            frontRightMotor.setPower(currentFrontRightPower);
            backLeftMotor.setPower(currentBackLeftPower);
            backRightMotor.setPower(currentBackRightPower);

            double flSpeed = RobotConstants.MECANUM_MOTORS_HAVE_ENCODERS
                    ? MotorUtils.getMotorVelocityTicksPerSec(frontLeftMotor) : 0.0;
            double frSpeed = RobotConstants.MECANUM_MOTORS_HAVE_ENCODERS
                    ? MotorUtils.getMotorVelocityTicksPerSec(frontRightMotor) : 0.0;
            double blSpeed = RobotConstants.MECANUM_MOTORS_HAVE_ENCODERS
                    ? MotorUtils.getMotorVelocityTicksPerSec(backLeftMotor) : 0.0;
            double brSpeed = RobotConstants.MECANUM_MOTORS_HAVE_ENCODERS
                    ? MotorUtils.getMotorVelocityTicksPerSec(backRightMotor) : 0.0;

            frontLeftFaultDetector.update(currentFrontLeftPower, frontLeftMotor.getCurrentPosition(), flSpeed);
            frontRightFaultDetector.update(currentFrontRightPower, frontRightMotor.getCurrentPosition(), frSpeed);
            backLeftFaultDetector.update(currentBackLeftPower, backLeftMotor.getCurrentPosition(), blSpeed);
            backRightFaultDetector.update(currentBackRightPower, backRightMotor.getCurrentPosition(), brSpeed);

            lastLeftSpeed = 0.5 * (flSpeed + blSpeed);
            lastRightSpeed = 0.5 * (frSpeed + brSpeed);

            checkDriveAutoEStop(eStop, frontLeftFaultDetector, frontRightFaultDetector,
                    backLeftFaultDetector, backRightFaultDetector);

            if (odometry != null) {
                odometry.update();
            }
        } else {
            currentLeftPower = ramp(currentLeftPower, targetLeftPower, dt);
            currentRightPower = ramp(currentRightPower, targetRightPower, dt);

            leftDriveMotor.setPower(currentLeftPower);
            rightDriveMotor.setPower(currentRightPower);

            lastLeftSpeed = MotorUtils.getMotorVelocityTicksPerSec(leftDriveMotor);
            lastRightSpeed = MotorUtils.getMotorVelocityTicksPerSec(rightDriveMotor);

            leftFaultDetector.update(currentLeftPower, leftDriveMotor.getCurrentPosition(), lastLeftSpeed);
            rightFaultDetector.update(currentRightPower, rightDriveMotor.getCurrentPosition(), lastRightSpeed);

            checkDriveAutoEStop(eStop, leftFaultDetector, rightFaultDetector);
        }

        reportTelemetry();
    }

    private void checkDriveAutoEStop(EmergencyStop eStop, MotorFaultDetector... detectors) {
        for (MotorFaultDetector detector : detectors) {
            if (detector == null) continue;
            boolean trigger = (RobotConstants.AUTO_ESTOP_DRIVETRAIN_ON_OVERCURRENT
                    && detector.isOvercurrentDetected())
                    || (RobotConstants.AUTO_ESTOP_DRIVETRAIN_ON_STALL && detector.isStallDetected());
            if (trigger) {
                String reason = detector.getName() + " " + detector.getPrimaryError().getShortLabel()
                        + ": " + detector.getErrorMessage();
                eStop.triggerSubsystem(SUBSYSTEM_KEY, reason);
                stopImmediately();
                return;
            }
        }
    }

    private double ramp(double current, double target, double dt) {
        boolean decelerating = Math.abs(target) < Math.abs(current)
                || (Math.signum(target) != Math.signum(current) && current != 0.0);

        double maxStep = (decelerating ? decelerationRate : accelerationRate) * dt;
        double error = target - current;

        if (Math.abs(error) <= maxStep) {
            return target;
        }
        return current + Math.copySign(maxStep, error);
    }

    private double clip(double power) {
        return Math.max(
                RobotConstants.DRIVETRAIN_OUTPUT_MIN,
                Math.min(RobotConstants.DRIVETRAIN_OUTPUT_MAX, power)
        );
    }

    private void reportTelemetry() {
        TelemetryServer server = TelemetryServer.getInstance();
        server.setSubsystemTelemetry("driveTrain", "mode", mecanumInstalled ? "mecanum+odo" : "tank");
        server.setSubsystemTelemetry("driveTrain", "speedMultiplier", speedMultiplier);

        if (mecanumInstalled) {
            server.setSubsystemTelemetry("driveTrain", "frontLeftPower", currentFrontLeftPower);
            server.setSubsystemTelemetry("driveTrain", "frontRightPower", currentFrontRightPower);
            server.setSubsystemTelemetry("driveTrain", "backLeftPower", currentBackLeftPower);
            server.setSubsystemTelemetry("driveTrain", "backRightPower", currentBackRightPower);
        } else {
            server.setSubsystemTelemetry("driveTrain", "leftPower", currentLeftPower);
            server.setSubsystemTelemetry("driveTrain", "rightPower", currentRightPower);
            server.setSubsystemTelemetry("driveTrain", "leftSpeed", lastLeftSpeed);
            server.setSubsystemTelemetry("driveTrain", "rightSpeed", lastRightSpeed);
        }

        server.setSubsystemTelemetry("driveTrain", "leftStall", isLeftStallDetected());
        server.setSubsystemTelemetry("driveTrain", "rightStall", isRightStallDetected());
        server.setSubsystemTelemetry("driveTrain", "leftEncoderFault", isLeftEncoderFault());
        server.setSubsystemTelemetry("driveTrain", "rightEncoderFault", isRightEncoderFault());
        server.setSubsystemTelemetry("driveTrain", "eStopped", isEStopped());
    }
}
