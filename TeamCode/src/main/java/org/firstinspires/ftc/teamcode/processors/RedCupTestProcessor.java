package org.firstinspires.ftc.teamcode.processors;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedCupTestProcessor extends OpenCvPipeline {

    public int blurValue = ((int) (5));
    private Mat blurGaussianMat = new Mat();

    public Scalar lowerYCrCb = new Scalar(0, 176.0, 0.0, 0.0);
    public Scalar upperYCrCb = new Scalar(157.0, 255.0, 255.0, 0.0);
    private Mat ycrcbMat = new Mat();
    private Mat ycrcbBinaryMat = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    public Scalar lineColor = new Scalar(255.0, 0.0, 0.0, 0.0);
    public int lineThickness = 2;

    private ArrayList<MatOfPoint> crosshair = new ArrayList<>();
    private Mat crosshairImage = new Mat();
    public int crosshairSize = 25;

    public int minArea = 50;
    public int maxArea = 70000;
    private ArrayList<MatOfPoint> crosshairByArea = new ArrayList<>();

    private MatOfPoint2f crosshairByArea2f = new MatOfPoint2f();
    private ArrayList<RotatedRect> crosshairByAreaRotRects = new ArrayList<>();

    public Scalar lineColor1 = new Scalar(0.0, 255.0, 0.0, 0.0);
    public int lineThickness1 = 3;

    private Mat inputRotRects = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.GaussianBlur(input, blurGaussianMat, new Size((6 * blurValue) + 1, (6 * blurValue) + 1), blurValue);

        Imgproc.cvtColor(blurGaussianMat, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMat, lowerYCrCb, upperYCrCb, ycrcbBinaryMat);

        contours.clear();
        hierarchy.release();
        Imgproc.findContours(ycrcbBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        input.copyTo(crosshairImage);

        Point crosshairPoint = new Point(((double) (input.cols())) / 2, ((double) (input.rows())) / 2);
        int scaleFactor = (input.rows() + input.cols()) / 2;

        int adjustedCrosshairSize = (crosshairSize * scaleFactor) / 100;

        Imgproc.line(crosshairImage, new Point(crosshairPoint.x - adjustedCrosshairSize, crosshairPoint.y), new Point(crosshairPoint.x + adjustedCrosshairSize, crosshairPoint.y), lineColor, lineThickness);
        Imgproc.line(crosshairImage, new Point(crosshairPoint.x, crosshairPoint.y - adjustedCrosshairSize), new Point(crosshairPoint.x, crosshairPoint.y + adjustedCrosshairSize), lineColor, lineThickness);

        crosshair.clear();

        for(MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);

            if(boundingRect.contains(crosshairPoint)) {
                crosshair.add(contour);
            }
        }

        crosshairByArea.clear();
        for(MatOfPoint contour : crosshair) {
            double area = Imgproc.contourArea(contour);
            if((area >= minArea) && (area <= maxArea)) {
                crosshairByArea.add(contour);
            }
        }

        crosshairByAreaRotRects.clear();
        for(MatOfPoint points : crosshairByArea) {
            crosshairByArea2f.release();
            points.convertTo(crosshairByArea2f, CvType.CV_32F);

            crosshairByAreaRotRects.add(Imgproc.minAreaRect(crosshairByArea2f));
        }

        input.copyTo(inputRotRects);
        for(RotatedRect rect : crosshairByAreaRotRects) {
            if(rect != null) {
                Point[] rectPoints = new Point[4];
                rect.points(rectPoints);
                MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

                Imgproc.polylines(inputRotRects, Collections.singletonList(matOfPoint), true, lineColor1, lineThickness1);
            }
        }

        return inputRotRects;
    }
}
