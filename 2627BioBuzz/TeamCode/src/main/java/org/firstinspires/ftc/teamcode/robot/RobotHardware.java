package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

/**
 * Central access point for every robot subsystem.
 *
 * Generated and maintained by FTC Code Utils.
 */
public class RobotHardware {
    private final LinearOpMode opMode;

    public DriveTrain driveTrain;
    public FlyWheel flyWheel;
    public Intake intake;

    public RobotHardware(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        driveTrain = new DriveTrain(opMode.hardwareMap);
        flyWheel = new FlyWheel(opMode.hardwareMap);
        intake = new Intake(opMode.hardwareMap);
    }

    public void updateAll() {
        driveTrain.update();
        flyWheel.update();
        intake.update();
    }
}
