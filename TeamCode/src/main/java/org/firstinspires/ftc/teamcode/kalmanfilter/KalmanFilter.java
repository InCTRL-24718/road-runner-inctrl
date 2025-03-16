package org.firstinspires.ftc.teamcode.kalmanfilter;

import com.acmerobotics.roadrunner.Pose2d;

public class KalmanFilter {
    private Pose2d estimate;
    private double processNoise, measurementNoise;

    public KalmanFilter(double processNoise, double measurementNoise) {
        this.estimate = new Pose2d(0, 0, 0);
        this.processNoise = processNoise; //how much we trust the model to be correct - lower is more weight
        this.measurementNoise = measurementNoise; //how much we trust the sensor measurement to be correct - lower is more weight
    }

    public void update(Pose2d imuPose, Pose2d odometryPose) {
        double k = processNoise / (processNoise + measurementNoise); // Kalman gain
        double headingDifference = odometryPose.heading.minus(imuPose.heading);
        double x = estimate.position.x + k * (odometryPose.position.x - imuPose.position.x);
        double y = estimate.position.y + k * (odometryPose.position.y - imuPose.position.y);
        double heading = estimate.heading.real + (k * headingDifference);

        estimate = new Pose2d(x, y, heading);
    }

    public Pose2d getEstimate() {
        return estimate;
    }
}
