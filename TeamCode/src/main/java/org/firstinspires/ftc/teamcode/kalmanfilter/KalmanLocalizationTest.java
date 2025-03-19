package org.firstinspires.ftc.teamcode.kalmanfilter;

import static org.firstinspires.ftc.teamcode.TankDrive.Params.inPerTick;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

//@Disabled
@TeleOp(name="Kalman Localization Test", group="Localizer Tests")
public class KalmanLocalizationTest extends LinearOpMode {
    private KalmanLocalizer localizer;
    private ThreeDeadWheelLocalizer localizer2;
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor par0 = null;
    private DcMotor par1 = null;
    double leftPower;
    double rightPower;
    private double powerMultiplier = 1.0;

    @Override

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0); // Robot starting at the origin

        //initialises motors and sets direction
        par0 = hardwareMap.get(DcMotor.class, "par0");
        par1 = hardwareMap.get(DcMotor.class, "par1");
        par0.setDirection(DcMotor.Direction.REVERSE);
        par1.setDirection(DcMotor.Direction.FORWARD);

        //initialises localizers
        localizer = new KalmanLocalizer(hardwareMap, inPerTick, initialPose);
        localizer2 = new ThreeDeadWheelLocalizer(hardwareMap, inPerTick, initialPose);

        FtcDashboard dashboard = FtcDashboard.getInstance(); //Initialises FTC dashboard on http://192.168.43.1:8080/dash

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //updates localizers and pose
            Pose2d kalmanPoseEstimate = localizer.getPoseEstimate();
            Pose2d odoPoseEstimate = localizer2.getPose();
            localizer.update();
            localizer2.update();

            leftPower  = gamepad1.left_stick_y ;
            rightPower = gamepad1.right_stick_y ;

            //allows live control of power multiplier
            if (gamepad1.dpad_up && powerMultiplier < 1.0) {
                powerMultiplier = powerMultiplier + 0.1;
            } else if (gamepad1.dpad_down && powerMultiplier > 0.2) {
                powerMultiplier = powerMultiplier - 0.1;
            } else if (gamepad1.triangle) {
                powerMultiplier = 1.0;
            }

            //send calculated power to wheels
            par0.setPower(powerMultiplier * leftPower);
            par1.setPower(powerMultiplier * rightPower);

            //driver station telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Power Multiplier", powerMultiplier);
            telemetry.addData(" ", " ");

            telemetry.addData("Filtered X", kalmanPoseEstimate.position.x);
            telemetry.addData("Filtered Y", kalmanPoseEstimate.position.y);
            telemetry.addData("Filtered Heading", Math.toDegrees(kalmanPoseEstimate.heading.real));

            telemetry.addData(" ", " ");
            telemetry.addData("Raw X", odoPoseEstimate.position.x);
            telemetry.addData("Raw Y", odoPoseEstimate.position.y);
            telemetry.addData("Raw Heading", Math.toDegrees(odoPoseEstimate.heading.real));

            telemetry.update();

            //FTC dashboard telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("X", kalmanPoseEstimate.position.x);
            packet.put("Y", kalmanPoseEstimate.position.y);
            packet.put("Heading", Math.toDegrees(kalmanPoseEstimate.heading.real));
            packet.put("Pose", kalmanPoseEstimate);

            packet.put("Raw X", odoPoseEstimate.position.x);
            packet.put("Raw Y", odoPoseEstimate.position.y);
            packet.put("Raw Heading", Math.toDegrees(odoPoseEstimate.heading.real));

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
