/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.processors.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

//currently causes eocvsim to crash
@TeleOp(name = "OpenCV Color Distance", group = "Vision")
public class ColorDistance extends LinearOpMode
{
    public static final double sampleWidthInMm = 8.4 * 10;
    public static double testObjWidth = 14.6 * 10; // todo: remove when done debugging

    public static final int imageWidth = 640;
    public static final double sensorWidthInMm = 3.58;
    public static final double focalLength = calcFocalLengthInPx(4.0, sensorWidthInMm);
    public static final double pFocalLength = calcFocalLengthInPx(4.38, 5.64); // todo: remove when done debugging

    @Override
    public void runOpMode()
    {
        ColorBlobLocatorProcessor redColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        ColorBlobLocatorProcessor yellowColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        ColorBlobLocatorProcessor blueColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors( redColorLocator, yellowColorLocator, blueColorLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .build();


        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> redBlobs = redColorLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> blueBlobs = blueColorLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> yellowBlobs = yellowColorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, redBlobs);
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, yellowBlobs);
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blueBlobs);

            telemetry.addLine(" Area Density Aspect  Center");
            telemetry.addData("amount of green blobs: ", blueBlobs.size()); // todo: remove when done debugging

            ColorBlobLocatorProcessor.Blob largestRedBlob;
            ColorBlobLocatorProcessor.Blob largestYellowBlob;
            ColorBlobLocatorProcessor.Blob largestBlueBlob;
            if (blueBlobs.isEmpty()) {
                largestBlueBlob = null;
                telemetry.addLine("did not find largest blue blob"); // todo: remove when done debugging
            }
            else {
                largestBlueBlob = blueBlobs.get(0);
                telemetry.addLine("found largest blue blob"); // todo: remove when done debugging
            }
            if (redBlobs.isEmpty()) {
                largestRedBlob = null;
            }
            else {
                largestRedBlob = redBlobs.get(0);
            }
            if (yellowBlobs.isEmpty()) {
                largestYellowBlob = null;
            }
            else {
                largestYellowBlob = yellowBlobs.get(0);
            }
            if (blueBlobs.isEmpty()) {
                largestBlueBlob = null;
            }
            else {
                largestBlueBlob = blueBlobs.get(0);
            }

            if (largestBlueBlob != null) {
                RotatedRect boxFit = largestBlueBlob.getBoxFit();
                int width = boxFit.boundingRect().width;
                double distance = getDistance(sampleWidthInMm, width, focalLength, sensorWidthInMm);
                telemetry.addData("blue blob distance in mm: ", distance);
            }

            if (largestRedBlob != null) {
                RotatedRect boxFit = largestRedBlob.getBoxFit();
                int width = boxFit.boundingRect().width;
                double distance = getDistance(sampleWidthInMm, width, focalLength, sensorWidthInMm);
                telemetry.addData("red blob distance in mm: ", distance);
            }

            if (largestYellowBlob != null) {
                RotatedRect boxFit = largestYellowBlob.getBoxFit();
                int width = boxFit.boundingRect().width;
                double distance = getDistance(sampleWidthInMm, width, focalLength, sensorWidthInMm);
                telemetry.addData("yellow blob distance in mm: ", distance);
            }

            if (largestBlueBlob != null) {
                RotatedRect boxFit = largestBlueBlob.getBoxFit();
                int width = boxFit.boundingRect().width;
                double distance = getDistance(sampleWidthInMm, width, focalLength, sensorWidthInMm);
                telemetry.addData("blue blob distance in mm: ", distance);
                }

            telemetry.update();
            sleep(50);
        }
    }

    private static double calcFocalLengthInPx(double focalLengthInMm, double sensorWidthInMm) {
        return (imageWidth * focalLengthInMm) / sensorWidthInMm;
    }
    private static double getDistance(double realObjWidth, double width, double focalLength, double sensorWidth) {
        return (imageWidth * realObjWidth * focalLength) / (width * sensorWidth);
    }

}
