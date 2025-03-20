package org.firstinspires.ftc.teamcode.kalmanfilter;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import static org.firstinspires.ftc.teamcode.TankDrive.Params.inPerTick;

public class KalmanLocalizer extends ThreeDeadWheelLocalizer {
    private final KalmanFilter filter;
    private ThreeDeadWheelLocalizer localizer;
    private IMU imu;

    public KalmanLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d pose) {
        super(hardwareMap, inPerTick, pose);
        localizer = new ThreeDeadWheelLocalizer(hardwareMap, inPerTick, pose);
        // TODO: tune filter parameters - process noise is too high: the robot will under react to large changes. measurement noise is too high: the robot will overreact to small changes,
        // TODO: add parameters to FTC dashboard configs for tuning
        filter = new KalmanFilter(0.01, 0.01); // kalman filter parameters - lower is more weight
        imu = hardwareMap.get(IMU.class, "imu");
    }

    public Pose2d getPoseEstimate() {
        Pose2d odoPose = localizer.getPose();
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Pose2d imuPose = new Pose2d(odoPose.position.x, odoPose.position.y, imuHeading);

        filter.update(imuPose, odoPose);
        return filter.getEstimate();
    }
}
