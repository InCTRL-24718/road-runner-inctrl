package org.firstinspires.ftc.teamcode.processors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.DrawCircleProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
@Disabled
@Autonomous(name = "OpenCV Circle", group = "Vision")
public class SimpleOpenCVCircle extends OpMode {

    private DrawCircleProcessor drawCircleProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        drawCircleProcessor = new DrawCircleProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), drawCircleProcessor);
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