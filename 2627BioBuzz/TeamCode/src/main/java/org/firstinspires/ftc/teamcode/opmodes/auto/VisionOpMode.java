package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@Autonomous(name = "Vision Op Mode")
public class VisionOpMode extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        robot.init();
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            robot.updateAll();
            telemetry.addData("AprilTags", aprilTag.getDetections().size());
            telemetry.update();
        }

        visionPortal.close();
    }
}
