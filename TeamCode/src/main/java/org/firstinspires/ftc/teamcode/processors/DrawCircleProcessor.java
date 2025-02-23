package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class DrawCircleProcessor implements VisionProcessor {

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.HSVToColor(new float[]{153.53F, 15.11F, 88.24F}));
        paint.setStyle(Paint.Style.FILL);
        canvas.drawCircle(10 * scaleBmpPxToCanvasPx, 10 * scaleBmpPxToCanvasPx, 10 * scaleBmpPxToCanvasPx, paint);
    }
}
