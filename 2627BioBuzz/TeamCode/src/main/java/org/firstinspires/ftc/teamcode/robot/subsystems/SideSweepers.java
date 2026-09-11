package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareNames;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * SideSweepers subsystem.
 *
 * Owns the hardware for this mechanism and exposes high-level control
 * methods. Call update() once per loop if this subsystem needs
 * continuous control (e.g. a PID loop).
 */
public class SideSweepers {

    private final CRServo sideSweeperLeft;
    private final CRServo sideSweeperRight;

    public SideSweepers(HardwareMap hardwareMap) {
        sideSweeperLeft = hardwareMap.get(
                    CRServo.class,
                    HardwareNames.SIDE_SWEEPER_LEFT
        );
        sideSweeperRight = hardwareMap.get(
                    CRServo.class,
                    HardwareNames.SIDE_SWEEPER_RIGHT
        );
    }

    public void startSweepers() {
        sideSweeperLeft.setPower(RobotConstants.SIDE_SWEEPERS_POWER);
        sideSweeperRight.setPower(-RobotConstants.SIDE_SWEEPERS_POWER);
    }

    /** Call this once per loop from the OpMode, if needed. */
    public void update() {
        // TODO: continuous control logic
    }
}
