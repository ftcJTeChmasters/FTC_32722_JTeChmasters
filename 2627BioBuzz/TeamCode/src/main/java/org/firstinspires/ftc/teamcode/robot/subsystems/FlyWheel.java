package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;


/**
 * FlyWheel subsystem.
 *
 * Owns the hardware for this mechanism and exposes high-level control
 * methods. Call update() once per loop if this subsystem needs
 * continuous control (e.g. a PID loop).
 */
public class FlyWheel {

    private final DcMotor flyWheelMotor;
    private final MotorUtils motorUtils = new MotorUtils();
    private final PIDController PID = new PIDController(0, 0, 0, 0);
    private double targetSpeed = 200;
    private boolean running = false;

    public FlyWheel(HardwareMap hardwareMap) {
        flyWheelMotor = hardwareMap.get(DcMotor.class, HardwareNames.FLY_WHEEL_MOTOR);
        TelemetryServer.getInstance().setTargetedPidListener((target, p, i, d, f, setpoint) -> {
            if ("FlyWheel".equals(target)) {
                PID.setPIDF(p, i, d, f);
                if (setpoint != null) {
                    targetSpeed = setpoint;
                }
            }
        });
    }

    public void start(double targetSpeed) {
        running = true; 
        this.targetSpeed = targetSpeed;
    }

    public void stop() {
        running = false;
    }
    
    public void toggle() {
        running = !running;
    }
        
    /** Call this once per loop from the OpMode, if needed. */
    public void update() {
        double currentSpeed = motorUtils.getMotorVelocityTicksPerSec(flyWheelMotor);
        if (running) {            
            double power = PID.calculate(targetSpeed, currentSpeed);
            flyWheelMotor.setPower(power);
        }
        TelemetryServer.getInstance().setPidGraph(targetSpeed, currentSpeed);
    }
}
