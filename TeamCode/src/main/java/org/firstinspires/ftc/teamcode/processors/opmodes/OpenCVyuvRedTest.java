package org.firstinspires.ftc.teamcode.processors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.YuvRedTestProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@Autonomous(name = "OpenCV yuv Red", group = "Vision")
public class OpenCVyuvRedTest extends OpMode {

    private VisionPortal visionPortal;
    private YuvRedTestProcessor yuvRedProcessor;

    @Override
    public void init() {
        yuvRedProcessor = new YuvRedTestProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), (VisionProcessor) yuvRedProcessor);
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
