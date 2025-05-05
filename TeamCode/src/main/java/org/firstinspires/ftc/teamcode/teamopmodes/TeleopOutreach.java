package org.firstinspires.ftc.teamcode.teamopmodes;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;

//@Disabled
@TeleOp(name="Outreach", group="Outreach")
public class TeleopOutreach extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
    private DcMotor rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
    private DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
    private Servo clawServo = hardwareMap.get(Servo.class, "claw");
    private Servo wristServo = hardwareMap.get(Servo.class, "wrist");
    private DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor"); private double distanceVal;
    private ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor"); private double redVal, greenVal, blueVal;
    private TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
    private double leftPower, rightPower, armPower;


    public void runOpMode() {

        //creates media players and designates their sources
        MediaPlayer mediaPlayer1 = MediaPlayer.create(hardwareMap.appContext, R.raw.rizz_sound_effect);
        MediaPlayer mediaPlayer2 = MediaPlayer.create(hardwareMap.appContext, R.raw.sponge_stank_noise);


        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status","Initialised");

        waitForStart();

        while (opModeIsActive()) {

            //uses left and right stick to control drive motors
            leftPower = gamepad1.left_stick_y; rightPower = gamepad1.right_stick_y;

            //controls arm motor with bumpers, checking that it doesn't go out of bounds relative to its starting position
            if (gamepad1.right_bumper && (arm.getCurrentPosition() <= 1000)) {armPower = 0.5;} else if (gamepad1.left_bumper && (arm.getCurrentPosition() >= -1000)) {armPower = -0.5;} else {armPower = 0;}

            //sets wrist position - x is up, y is down
            if (gamepad1.x) {wristServo.setPosition(1);} else if (gamepad1.y) {wristServo.setPosition(0);}

            //sets claw position - a is open, b is closed
            if (gamepad1.a) {clawServo.setPosition(1);} else if (gamepad1.b) {clawServo.setPosition(0);}

            //sets motor powers
            leftDrive.setPower(leftPower); rightDrive.setPower(rightPower); arm.setPower(armPower);


            //unleashes rizz when button is pressed
            if (touchSensor.isPressed() && !mediaPlayer2.isPlaying() && !mediaPlayer1.isPlaying()) {mediaPlayer1.start();}

            //plays audio when too close to object
            distanceVal = distanceSensor.getDistance(DistanceUnit.INCH);
            if ((distanceVal <= 3) && !mediaPlayer2.isPlaying() && !mediaPlayer1.isPlaying()) {mediaPlayer2.start();}

            //changes LED colour based on colour sensor
            redVal = colorSensor.red();
            greenVal = colorSensor.green();
            blueVal = colorSensor.blue();
            
            if ((redVal >= 180) && (greenVal <= 50) && (blueVal >= 50)) {gamepad1.setLedColor(255,0,0, 1000);}
            else if ((redVal <= 50) && (greenVal >= 180) && (blueVal <= 50)) {gamepad1.setLedColor(0,255,0, 1000);}
            else if ((redVal <= 50) && (greenVal <= 50) && (blueVal >= 180)) {gamepad1.setLedColor(0,0,255, 1000);}


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Button pressed:", touchSensor.isPressed());
            telemetry.addData("About to crash?", mediaPlayer2.isPlaying());
            telemetry.update();
        }
    }
}
