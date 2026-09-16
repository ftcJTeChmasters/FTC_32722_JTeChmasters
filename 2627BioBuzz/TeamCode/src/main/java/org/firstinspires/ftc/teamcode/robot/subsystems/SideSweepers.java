package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareNames;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.util.NonBlockingWait;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;

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
    NonBlockingWait wait = new NonBlockingWait();

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
    
    public void extendSweepers() {
        TelemetryServer.getInstance().setSubsystemTelemetry("Side Sweepers", "State", "Extending sweepers (1/3)");
        sideSweeperLeft.setPower(-1);
        sideSweeperRight.setPower(1);
        TelemetryServer.getInstance().setSubsystemTelemetry("Side Sweepers", "State", "Extending sweepers (2/3)");
        wait.start(RobotConstants.SIDE_SWEEPERS_EXTEND_TIME_MS);
        while (!wait.isDone()) {
            
        }
        sideSweeperLeft.setPower(0);
        sideSweeperRight.setPower(0);
        TelemetryServer.getInstance().setSubsystemTelemetry("Side Sweepers", "State", "Sweepers extended (3/3)");
    }

    public void startSweepers() {
        TelemetryServer.getInstance().setSubsystemTelemetry("Side Sweepers", "State", "Starting sweepers");
        sideSweeperLeft.setPower(RobotConstants.SIDE_SWEEPERS_POWER);
        sideSweeperRight.setPower(-RobotConstants.SIDE_SWEEPERS_POWER);
        TelemetryServer.getInstance().setSubsystemTelemetry("Side Sweepers", "State", "Sweeping");
    }

    /** Call this once per loop from the OpMode, if needed. */
    public void update() {
        // TODO: continuous control logic
    }
}
