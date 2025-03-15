package org.firstinspires.ftc.teamcode.processors;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueTestProcessor extends OpenCvPipeline {

    public int blurValue = 2;
    private final Mat blurGaussianMat = new Mat();

    public Scalar lowerYCrCb = new Scalar(44.0, 0.0, 163.0, 0.0);
    public Scalar upperYCrCb = new Scalar(189.0, 139.0, 255.0, 0.0);
    private final Mat ycrcbMat = new Mat();
    private final Mat ycrcbBinaryMat = new Mat();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    private final Mat hierarchy = new Mat();

    public int minArea = 50;
    public int maxArea = 100000;
    private final ArrayList<MatOfPoint> contoursByArea = new ArrayList<>();

    public Scalar lineColor = new Scalar(173.0, 0.0, 175.0, 0.0);
    public int lineThickness = 1;

    private final ArrayList<MatOfPoint> crosshair = new ArrayList<>();
    private final Mat crosshairImage = new Mat();
    public int crosshairSize = 32;

    private final MatOfPoint2f crosshair2f = new MatOfPoint2f();
    private final ArrayList<RotatedRect> crosshairRotRects = new ArrayList<>();

    public Scalar lineColor1 = new Scalar(0.0, 255.0, 0.0, 0.0);
    public int lineThickness1 = 2;

    public Scalar lineColor2 = new Scalar(255.0, 255.0, 255.0, 0.0);
    public int lineThickness2 = 2;

    private final Mat inputContours = new Mat();

    private final Mat inputContoursRotRects = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.GaussianBlur(input, blurGaussianMat, new Size((6 * blurValue) + 1, (6 * blurValue) + 1), blurValue);

        Imgproc.cvtColor(blurGaussianMat, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMat, lowerYCrCb, upperYCrCb, ycrcbBinaryMat);

        contours.clear();
        hierarchy.release();
        Imgproc.findContours(ycrcbBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contoursByArea.clear();
        for(MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if((area >= minArea) && (area <= maxArea)) {
                contoursByArea.add(contour);
            }
        }

        input.copyTo(crosshairImage);

        Point crosshairPoint = new Point(((double) (input.cols())) / 2, ((double) (input.rows())) / 2);
        int scaleFactor = (input.rows() + input.cols()) / 2;

        int adjustedCrosshairSize = (crosshairSize * scaleFactor) / 100;

        Imgproc.line(crosshairImage, new Point(crosshairPoint.x - adjustedCrosshairSize, crosshairPoint.y), new Point(crosshairPoint.x + adjustedCrosshairSize, crosshairPoint.y), lineColor, lineThickness);
        Imgproc.line(crosshairImage, new Point(crosshairPoint.x, crosshairPoint.y - adjustedCrosshairSize), new Point(crosshairPoint.x, crosshairPoint.y + adjustedCrosshairSize), lineColor, lineThickness);

        crosshair.clear();

        for(MatOfPoint contour : contoursByArea) {
            Rect boundingRect = Imgproc.boundingRect(contour);

            if(boundingRect.contains(crosshairPoint)) {
                crosshair.add(contour);
            }
        }

        crosshairRotRects.clear();
        for(MatOfPoint points : crosshair) {
            crosshair2f.release();
            points.convertTo(crosshair2f, CvType.CV_32F);

            crosshairRotRects.add(Imgproc.minAreaRect(crosshair2f));
        }

        input.copyTo(inputContours);
        Imgproc.drawContours(inputContours, crosshair, -1, lineColor2, lineThickness2);

        inputContours.copyTo(inputContoursRotRects);
        for(RotatedRect rect : crosshairRotRects) {
            if(rect != null) {
                Point[] rectPoints = new Point[4];
                rect.points(rectPoints);
                MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

                Imgproc.polylines(inputContoursRotRects, Collections.singletonList(matOfPoint), true, lineColor1, lineThickness1);
            }
        }

        return inputContoursRotRects;
    }
}