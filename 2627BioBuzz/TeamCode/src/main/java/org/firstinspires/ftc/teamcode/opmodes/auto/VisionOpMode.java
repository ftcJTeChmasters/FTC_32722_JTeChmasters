package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.debug.RobotMonitor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Autonomous mode demonstrating AprilTag detection.
 * <p>
 * Initializes the vision portal with an AprilTag processor and reports
 * the number of detected tags each loop on the Driver Station viewer.
 */
@Autonomous(name = "Vision Op Mode")
public class VisionOpMode extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this);
    private final RobotMonitor monitor = new RobotMonitor(this, robot);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        monitor.init();
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .build();

        waitForStart();
        monitor.start();
        monitor.setCurrentTask("Scanning AprilTags");

        while (opModeIsActive()) {
            monitor.setDsExtra("AprilTags", aprilTag.getDetections().size());
            monitor.update();
        }

        visionPortal.close();
        monitor.stop();
    }
}
