package org.firstinspires.ftc.teamcode.processors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.RedCupTestProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@Autonomous(name = "OpenCV Red Cup", group = "Vision")
public class OpenCVRedCupTest extends OpMode {

    private RedCupTestProcessor redCupTestProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        redCupTestProcessor = new RedCupTestProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), (VisionProcessor) redCupTestProcessor);
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
