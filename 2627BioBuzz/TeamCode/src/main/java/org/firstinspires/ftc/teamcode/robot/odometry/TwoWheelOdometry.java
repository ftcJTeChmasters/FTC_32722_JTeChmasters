package org.firstinspires.ftc.teamcode.robot.odometry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;

/**
 * Two-wheel dead-wheel odometry localizer paired with the Control Hub IMU.
 * <p>
 * Tracks robot field position {@code (xInches, yInches)} and {@code headingDegrees}
 * using:
 * <ul>
 *   <li>One forward-aligned dead-wheel encoder pod ({@link HardwareNames#ODOMETRY_FORWARD_POD})</li>
 *   <li>One strafe-aligned dead-wheel encoder pod ({@link HardwareNames#ODOMETRY_STRAFE_POD})</li>
 *   <li>Control Hub built-in {@link IMU} ({@link HardwareNames#IMU}) for heading</li>
 * </ul>
 * <p>
 * Configured pod relative positions in {@link RobotConstants} are used to cancel
 * rotational coupling from off-center pod mounting, and pose-exponential arc
 * integration prevents heading-turn drift.
 */
public class TwoWheelOdometry {

    private final DcMotor forwardPod;
    private final DcMotor strafePod;
    private final IMU imu;

    private int lastForwardTicks;
    private int lastStrafeTicks;
    private double lastRawHeadingRadians;

    private double xInches = 0.0;
    private double yInches = 0.0;
    private double headingRadians = 0.0;

    private int currentForwardTicks = 0;
    private int currentStrafeTicks = 0;

    public TwoWheelOdometry(HardwareMap hardwareMap) {
        forwardPod = hardwareMap.get(DcMotor.class, HardwareNames.ODOMETRY_FORWARD_POD);
        strafePod = hardwareMap.get(DcMotor.class, HardwareNames.ODOMETRY_STRAFE_POD);
        imu = hardwareMap.get(IMU.class, HardwareNames.IMU);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RobotConstants.IMU_LOGO_FACING_DIRECTION,
                RobotConstants.IMU_USB_FACING_DIRECTION
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Software baseline capture (avoids STOP_AND_RESET_ENCODER on shared motor ports)
        currentForwardTicks = forwardPod.getCurrentPosition();
        currentStrafeTicks = strafePod.getCurrentPosition();
        lastForwardTicks = currentForwardTicks;
        lastStrafeTicks = currentStrafeTicks;
        lastRawHeadingRadians = readRawImuYawRadians();
    }

    /**
     * Updates the estimated field position and heading. Call once per loop.
     */
    public void update() {
        currentForwardTicks = forwardPod.getCurrentPosition();
        currentStrafeTicks = strafePod.getCurrentPosition();
        double rawHeadingRadians = readRawImuYawRadians();

        int deltaForwardTicks = currentForwardTicks - lastForwardTicks;
        int deltaStrafeTicks = currentStrafeTicks - lastStrafeTicks;
        double deltaTheta = wrapRadians(rawHeadingRadians - lastRawHeadingRadians);

        lastForwardTicks = currentForwardTicks;
        lastStrafeTicks = currentStrafeTicks;
        lastRawHeadingRadians = rawHeadingRadians;

        double deltaForwardPodInches = deltaForwardTicks
                * RobotConstants.ODOMETRY_FORWARD_INCHES_PER_TICK
                * RobotConstants.ODOMETRY_FORWARD_POD_DIRECTION;

        double deltaStrafePodInches = deltaStrafeTicks
                * RobotConstants.ODOMETRY_STRAFE_INCHES_PER_TICK
                * RobotConstants.ODOMETRY_STRAFE_POD_DIRECTION;

        // Compensate for each pod's relative offset from the robot tracking center:
        //   v_x = -omega * y  =>  deltaForwardRobot = deltaPodFwd + y_fwd * dTheta
        //   v_y = +omega * x  =>  deltaStrafeRobot  = deltaPodStrafe - x_strafe * dTheta
        double deltaForwardRobot = deltaForwardPodInches
                + RobotConstants.ODOMETRY_FORWARD_POD_Y_INCHES * deltaTheta;
        double deltaStrafeRobot = deltaStrafePodInches
                - RobotConstants.ODOMETRY_STRAFE_POD_X_INCHES * deltaTheta;

        // Exact pose-exponential integration in the robot frame
        double sineTerm;
        double cosineTerm;
        if (Math.abs(deltaTheta) < RobotConstants.ODOMETRY_SMALL_ANGLE_THRESHOLD) {
            sineTerm = 1.0 - (deltaTheta * deltaTheta) / 6.0;
            cosineTerm = deltaTheta / 2.0;
        } else {
            sineTerm = Math.sin(deltaTheta) / deltaTheta;
            cosineTerm = (1.0 - Math.cos(deltaTheta)) / deltaTheta;
        }

        double deltaXRobot = sineTerm * deltaForwardRobot - cosineTerm * deltaStrafeRobot;
        double deltaYRobot = cosineTerm * deltaForwardRobot + sineTerm * deltaStrafeRobot;

        // Rotate robot-frame displacement into field coordinates using previous heading
        double cosHeading = Math.cos(headingRadians);
        double sinHeading = Math.sin(headingRadians);
        xInches += deltaXRobot * cosHeading - deltaYRobot * sinHeading;
        yInches += deltaXRobot * sinHeading + deltaYRobot * cosHeading;
        headingRadians = wrapRadians(headingRadians + deltaTheta);

        reportTelemetry();
    }

    /** Resets estimated pose to (0, 0, 0°). */
    public void resetPose() {
        setPose(0.0, 0.0, 0.0);
    }

    /**
     * Sets the current field pose to a known position and heading.
     *
     * @param xInches        field X in inches
     * @param yInches        field Y in inches
     * @param headingDegrees heading in degrees (0° = +X, CCW positive)
     */
    public void setPose(double xInches, double yInches, double headingDegrees) {
        this.xInches = xInches;
        this.yInches = yInches;
        this.headingRadians = wrapRadians(Math.toRadians(headingDegrees));

        currentForwardTicks = forwardPod.getCurrentPosition();
        currentStrafeTicks = strafePod.getCurrentPosition();
        lastForwardTicks = currentForwardTicks;
        lastStrafeTicks = currentStrafeTicks;
        lastRawHeadingRadians = readRawImuYawRadians();
    }

    public double getXInches() {
        return xInches;
    }

    public double getYInches() {
        return yInches;
    }

    public double getHeadingRadians() {
        return headingRadians;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(headingRadians);
    }

    public int getForwardTicks() {
        return currentForwardTicks;
    }

    public int getStrafeTicks() {
        return currentStrafeTicks;
    }

    private double readRawImuYawRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private static double wrapRadians(double angleRadians) {
        double twoPi = 2.0 * Math.PI;
        double wrapped = (angleRadians + Math.PI) % twoPi;
        if (wrapped < 0) {
            wrapped += twoPi;
        }
        return wrapped - Math.PI;
    }

    private void reportTelemetry() {
        double headingDeg = getHeadingDegrees();
        TelemetryServer server = TelemetryServer.getInstance();
        server.setPosition(xInches, yInches, headingDeg);
        server.setSubsystemTelemetry("odometry", "xInches", xInches);
        server.setSubsystemTelemetry("odometry", "yInches", yInches);
        server.setSubsystemTelemetry("odometry", "headingDegrees", headingDeg);
        server.setSubsystemTelemetry("odometry", "forwardTicks", currentForwardTicks);
        server.setSubsystemTelemetry("odometry", "strafeTicks", currentStrafeTicks);
    }
}
