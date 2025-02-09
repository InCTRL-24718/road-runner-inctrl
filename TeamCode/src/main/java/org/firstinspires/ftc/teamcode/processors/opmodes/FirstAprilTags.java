package org.firstinspires.ftc.teamcode.processors.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "April Tags", group = "Vision")
public class FirstAprilTags extends OpMode {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private String aprilTagName;
    private int lastAprilTagIdCode;
    private AprilTagDetection aprilTagDetection;

    @Override
    public void init() {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), (VisionProcessor) aprilTagProcessor);

    }

    @Override
    public void init_loop() {

        /* //Will only detect one April Tag
        int aprilTagIdCode = aprilTagDetection.id;
        telemetry.addData("April Tag ID Code", aprilTagIdCode);
        telemetry.update();
        */

        List<AprilTagDetection> aprilTagDetections;  // list of all detections
        int aprilTagIdCode = 0;                      // ID code of current detection, in for() loop

        // Get a list of AprilTag detections.
        aprilTagDetections = aprilTagProcessor.getDetections();

        // Cycle through through the list and process each AprilTag.
        for (AprilTagDetection aprilTagDetection : aprilTagDetections) {

            if (aprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                aprilTagIdCode = aprilTagDetection.id;
                aprilTagName = aprilTagDetection.metadata.name;
                double tagPoseX = aprilTagDetection.ftcPose.x;
                double tagPoseY = aprilTagDetection.ftcPose.y;
                double tagPoseZ = aprilTagDetection.ftcPose.z;
                double tagPosePitch = aprilTagDetection.ftcPose.pitch;
                double tagPoseRoll = aprilTagDetection.ftcPose.roll;
                double tagPoseYaw = aprilTagDetection.ftcPose.yaw;
                double tagPoseRange = aprilTagDetection.ftcPose.range;
                double tagPoseBearing = aprilTagDetection.ftcPose.bearing;
                double tagPoseElevation = aprilTagDetection.ftcPose.elevation;

                telemetry.addData("April Tag ID Code", aprilTagIdCode);
                telemetry.addData("April Tag Name", aprilTagName);
                telemetry.addData("Tag Pose X", tagPoseX);
                telemetry.addData("Tag Pose Y", tagPoseY);
                telemetry.addData("Tag Pose Z", tagPoseZ);
                telemetry.addData("Tag Pose Pitch", tagPosePitch);
                telemetry.addData("Tag Pose Roll", tagPoseRoll);
                telemetry.addData("Tag Pose Yaw", tagPoseYaw);
                telemetry.addData("Tag Pose Range", tagPoseRange);
                telemetry.addData("Tag Pose Bearing", tagPoseBearing);
                telemetry.addData("Tag Pose Elevation", tagPoseElevation);
                telemetry.update();

                // Now take action based on this tag's ID code, or store info for later action.
            }
        }
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {

    }
}
