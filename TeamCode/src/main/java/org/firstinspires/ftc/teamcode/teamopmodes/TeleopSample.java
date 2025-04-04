//general teleop, used in regionals

package org.firstinspires.ftc.teamcode.teamopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class TeleopSample extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private Servo wristServo = null;
    private Servo clawServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "par0");
        rightDrive = hardwareMap.get(DcMotor.class, "par1");
        armMotor = hardwareMap.get(DcMotor.class, "Arm");
        wristServo = hardwareMap.get(Servo.class, "Wrist");
        clawServo = hardwareMap.get(Servo.class, "Claw");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawServo.setPosition(0);
        wristServo.setPosition(0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;
            double ticksPerMotorRevolution = 28.0;
            double gearReduction = 18.9;
            double wheelCircumference = 101.6 * Math.PI;

            double ticksPerWheelRevolution = ticksPerMotorRevolution * gearReduction;
            double ticksPerMillimeter = ticksPerWheelRevolution / wheelCircumference;
            double ticksPerInch = ticksPerMillimeter * 25.4;

            leftPower  = gamepad1.left_stick_y ;
            rightPower = gamepad1.right_stick_y ;
            if (gamepad1.right_bumper) {
                armMotor.setPower(0.5);
            } else if (gamepad1.left_bumper) {
                armMotor.setPower(-0.5);
            }
            else {
                armMotor.setPower(0);
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            if (gamepad1.dpad_up) {
                armMotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                armMotor.setPower(-1);
            } else {
                armMotor.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
