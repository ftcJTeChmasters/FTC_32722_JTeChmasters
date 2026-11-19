package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * DriveTrain subsystem.
 *
 * Owns the hardware for this mechanism and exposes high-level control
 * methods. Call update() once per loop if this subsystem needs
 * continuous control (e.g. a PID loop).
 */
public class DriveTrain {
    private final DcMotor leftDriveMotor;
    private final DcMotor rightDriveMotor;
    private final MotorUtils motorUtils = new MotorUtils();

    public DriveTrain(HardwareMap hardwareMap) {
        leftDriveMotor = hardwareMap.get(DcMotor.class, HardwareNames.LEFT_DRIVE_MOTOR);
        rightDriveMotor = hardwareMap.get(DcMotor.class, HardwareNames.RIGHT_DRIVE_MOTOR);
    }

    public void setLeftPower(double power) {
        leftDriveMotor.setPower(power);
    }

    public void setRightPower(double power) {
        rightDriveMotor.setPower(power);
    }

    public double getLeftSpeed() {
        return motorUtils.getMotorVelocityTicksPerSec(leftDriveMotor);
    }

    public double getRightSpeed() {
        return motorUtils.getMotorVelocityTicksPerSec(rightDriveMotor);
    }

    public void setPowers(double leftPower, double rightPower) {
        leftDriveMotor.setPower(leftPower);
        rightDriveMotor.setPower(rightPower);
    }

    public void getSpeeds(double[] speeds) {
        speeds[0] = getLeftSpeed();
        speeds[1] = getRightSpeed();
    }

    public void stop() {
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
    }

    /** Call this once per loop from the OpMode, if needed. */
    public void update() {
        // TODO: continuous control logic
    }
}
