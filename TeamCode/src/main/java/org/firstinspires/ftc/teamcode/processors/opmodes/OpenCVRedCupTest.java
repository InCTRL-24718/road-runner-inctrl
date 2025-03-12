package org.firstinspires.ftc.teamcode.processors.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.RedCupTestProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.openftc.easyopencv.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "OpenCV Red Cup", group = "Vision")
public class OpenCVRedCupTest extends OpMode {
    private VisionPortal visionPortal;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        RedCupTestProcessor redCupTest = new RedCupTestProcessor();
        webcam.setPipeline(redCupTest);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.SIDEWAYS_RIGHT, OpenCvWebcam.StreamFormat.MJPEG);
            }

            @Override
            public void onError(int errorCode) {}
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
    }
}