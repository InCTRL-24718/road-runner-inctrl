package org.firstinspires.ftc.teamcode.kalmanfilter;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

public class KalmanLocalizer extends ThreeDeadWheelLocalizer {
    private final KalmanFilter filter;
    private ThreeDeadWheelLocalizer localizer;
    private IMU imu;

    public KalmanLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d pose) {
        super(hardwareMap, inPerTick, pose);
        filter = new KalmanFilter(0.01, 0.01); // kalman filter parameters - lower is more weight
        imu = hardwareMap.get(IMU.class, "imu");
    }

    public Pose2d getPoseEstimate() {
        Pose2d odoPose = localizer.getPose();
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Pose2d imuPose = new Pose2d(odoPose.position.x, odoPose.position.y, imuHeading);

        filter.update(odoPose, imuPose);
        return filter.getEstimate();
    }
}
