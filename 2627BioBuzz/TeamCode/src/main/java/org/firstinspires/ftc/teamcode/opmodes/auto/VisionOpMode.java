package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

/**
 * Autonomous mode demonstrating AprilTag detection.
 * <p>
 * Initializes the vision portal with an AprilTag processor and reports
 * the number of detected tags each loop. Useful for testing camera
 * configuration and tag detection.
 */
@Autonomous(name = "Vision Op Mode")
public class VisionOpMode extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

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
