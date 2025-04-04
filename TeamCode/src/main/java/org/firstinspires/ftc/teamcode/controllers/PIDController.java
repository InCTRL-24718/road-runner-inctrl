//ALL VARIABLES ARE PLACEHOLDER VALUES

package org.firstinspires.ftc.teamcode.controllers;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    double Kp = 0; //tbt
    double Ki = 0; //stays as 0
    double Kd = 0; //tbt
    double IntegralSum = 0;
    ElapsedTime timer = new ElapsedTime();

    private DcMotor armMotor = null;
    public PIDController(double Kp, double Ki, double Kd) {

    }
    public double update(double referencePosition, double currentState) {
        double error = referencePosition - currentState;
        double derivative = error / timer.seconds();
        IntegralSum = IntegralSum + (error * timer.seconds());
        double output = (Kp * error) + (Ki * IntegralSum) + (Kd * derivative);
        return output;
    }
}
