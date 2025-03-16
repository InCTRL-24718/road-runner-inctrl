package org.firstinspires.ftc.teamcode.processors.opmodes.kalmanfilter;

import com.acmerobotics.roadrunner.Pose2d;

public class KalmanFilter {
    private Pose2d estimate;
    private double processNoise, measurementNoise;

    public KalmanFilter(double processNoise, double measurementNoise) {
        this.estimate = new Pose2d(0, 0, 0);
        this.processNoise = processNoise;
        this.measurementNoise = measurementNoise;
    }

    public void update(Pose2d encoderPose, Pose2d odometryPose) {
        double k = processNoise / (processNoise + measurementNoise);

        Pose2d predictedEstimate = estimate;

        double headingDifference = odometryPose.heading.minus(encoderPose.heading);
        double x = estimate.position.x + k * (odometryPose.position.x - encoderPose.position.x);
        double y = estimate.position.y + k * (odometryPose.position.y - encoderPose.position.y);
        double heading = estimate.heading.real + (k * headingDifference);

        estimate = new Pose2d(x, y, heading);
    }

    public Pose2d getEstimate() {
        return estimate;
    }
}
