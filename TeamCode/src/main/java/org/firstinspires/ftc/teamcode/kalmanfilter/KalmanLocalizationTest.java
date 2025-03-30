package org.firstinspires.ftc.teamcode.kalmanfilter;

import static org.firstinspires.ftc.teamcode.TankDrive.Params.inPerTick;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.vision.VisionPortal;

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

        //initialises motors and sets direction and zero power behavior
        par0 = hardwareMap.get(DcMotor.class, "par0");
        par1 = hardwareMap.get(DcMotor.class, "par1");
        par0.setDirection(DcMotor.Direction.REVERSE);
        par1.setDirection(DcMotor.Direction.FORWARD);
        par0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        par1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        localizer = new KalmanLocalizer(hardwareMap, inPerTick, initialPose); //initialises kalman localizer

        FtcDashboard dashboard = FtcDashboard.getInstance(); //Initialises FTC dashboard on http://192.168.43.1:8080/dash

        //initialises camera without processors via vision portal
        VisionPortal portal = new VisionPortal.Builder()
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        FtcDashboard.getInstance().startCameraStream(portal, 15); //starts camera stream on dashboard

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //updates localizers and pose
            Pose2d kalmanPoseEstimate = localizer.getPoseEstimate();
            localizer.update();

            leftPower  = gamepad1.left_stick_y ;
            rightPower = gamepad1.right_stick_y ;

            //TODO: Use DPad up and down to cycle through preset power values, e.g. 0.2, 0.4, 0.6, 0.8, 1
            //allows live control of power multiplier
            if (gamepad1.dpad_up && powerMultiplier < 1.0) {
                powerMultiplier = powerMultiplier + 0.05;
            } else if (gamepad1.dpad_down && powerMultiplier > 0.2) {
                powerMultiplier = powerMultiplier - 0.05;
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

            telemetry.update();

            //FTC dashboard telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("X", kalmanPoseEstimate.position.x);
            packet.put("Y", kalmanPoseEstimate.position.y);
            packet.put("Heading", Math.toDegrees(kalmanPoseEstimate.heading.real));

            packet.fieldOverlay().setStroke("#3F51B5"); //sets colour of robot on dashboard
            Drawing.drawRobot(packet.fieldOverlay(), kalmanPoseEstimate); //draws robot on FTC dashboard

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
