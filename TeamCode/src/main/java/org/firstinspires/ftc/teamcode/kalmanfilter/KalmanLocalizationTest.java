package org.firstinspires.ftc.teamcode.kalmanfilter;

import static org.firstinspires.ftc.teamcode.TankDrive.Params.inPerTick;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

public class KalmanLocalizationTest extends LinearOpMode {
    private KalmanLocalizer localizer;
    private ThreeDeadWheelLocalizer localizer2;
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor par0 = null;
    private DcMotor par1 = null;

    @Override

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0); // Robot starting at the origin
        localizer = new KalmanLocalizer(hardwareMap, inPerTick, initialPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        par0 = hardwareMap.get(DcMotor.class, "par0");
        par1 = hardwareMap.get(DcMotor.class, "par1");
        par0.setDirection(DcMotor.Direction.REVERSE);
        par1.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            localizer.update();
            Pose2d kalmanPoseEstimate = localizer.getPoseEstimate();
            Pose2d odoPoseEstimate = localizer2.getPose();

            double leftPower;
            double rightPower;

            leftPower  = gamepad1.left_stick_y ;
            rightPower = gamepad1.right_stick_y ;

            // Send calculated power to wheels
            par0.setPower(0.8 * leftPower);
            par1.setPower(0.8 * rightPower);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Filtered X", kalmanPoseEstimate.position.x);
            telemetry.addData("Filtered Y", kalmanPoseEstimate.position.y);
            telemetry.addData("Filtered Heading", Math.toDegrees(kalmanPoseEstimate.heading.real));

            telemetry.addData("Raw X", odoPoseEstimate.position.x);
            telemetry.addData("Raw Y", odoPoseEstimate.position.y);
            telemetry.addData("Raw Heading", Math.toDegrees(odoPoseEstimate.heading.real));

            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("X", kalmanPoseEstimate.position.x);
            packet.put("Y", kalmanPoseEstimate.position.y);
            packet.put("Heading", Math.toDegrees(kalmanPoseEstimate.heading.real));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
