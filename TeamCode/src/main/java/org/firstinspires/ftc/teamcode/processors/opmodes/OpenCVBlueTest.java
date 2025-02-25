package org.firstinspires.ftc.teamcode.processors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.BlueTestProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@Autonomous(name = "OpenCVBlueTest", group = "Vision")
public class OpenCVBlueTest extends OpMode {

    private VisionPortal visionPortal;
    private BlueTestProcessor blueProcessor;

    @Override
    public void init() {
        blueProcessor = new BlueTestProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), (VisionProcessor) blueProcessor);
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