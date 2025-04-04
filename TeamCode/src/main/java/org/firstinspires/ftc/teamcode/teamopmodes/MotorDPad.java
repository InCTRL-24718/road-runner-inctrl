//motor movement through d-pad

package org.firstinspires.ftc.teamcode.teamopmodes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="Motor Test", group="Linear OpMode")
public class MotorDPad extends LinearOpMode {
    private DcMotor singleMotor = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        singleMotor = hardwareMap.get(DcMotor.class, "par0");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                singleMotor.setPower(0.5);
            } else if (gamepad1.dpad_right) {
                singleMotor.setPower(-0.5);
            }
        }
    }

}
