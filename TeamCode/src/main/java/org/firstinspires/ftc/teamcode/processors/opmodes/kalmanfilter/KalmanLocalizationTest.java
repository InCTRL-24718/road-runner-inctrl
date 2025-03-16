package org.firstinspires.ftc.teamcode.processors.opmodes.kalmanfilter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class KalmanLocalizationTest extends LinearOpMode {
    private KalmanLocalizer localizer;

    @Override

    public void runOpMode() {
        double inPerTick = 0.001; // Example: Encoder ticks per inch
        Pose2d initialPose = new Pose2d(0, 0, 0); // Robot starting at the origin
        localizer = new KalmanLocalizer(hardwareMap, inPerTick, initialPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        while (opModeIsActive()) {
            localizer.update();
            Pose2d poseEstimate = localizer.getPoseEstimate();

            telemetry.addData("Filtered X", poseEstimate.position.x);
            telemetry.addData("Filtered Y", poseEstimate.position.y);
            telemetry.addData("Filtered Heading", Math.toDegrees(poseEstimate.heading.real));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("X", poseEstimate.position.x);
            packet.put("Y", poseEstimate.position.y);
            packet.put("Heading", Math.toDegrees(poseEstimate.heading.real));
            dashboard.sendTelemetryPacket(packet);
        }


    }
}
