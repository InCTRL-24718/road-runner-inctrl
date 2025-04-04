// hourglass position using roadrunner

package org.firstinspires.ftc.teamcode.teamopmodes;

import static org.firstinspires.ftc.teamcode.TankDrive.Params.inPerTick;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.kalmanfilter.KalmanLocalizer;

//@Disabled
@Autonomous(name="RR Test Auto", group="RR Tests")
public class RoadrunnerTestOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor par0 = null;
    private DcMotor par1 = null;
    Pose2d initialPose = new Pose2d(-36, 60, Math.toRadians(0)); //facing right

    @Override
    public void runOpMode() {

        par0 = hardwareMap.get(DcMotor.class, "par0");
        par1 = hardwareMap.get(DcMotor.class, "par1");

        KalmanLocalizer localizer = new KalmanLocalizer(hardwareMap, inPerTick, initialPose); //uses kalman localizer (3 deadwheel + imu)
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder myTraj = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(40, 60), Math.toRadians(0))
                .splineTo(new Vector2d(-30, 20), Math.toRadians(90))
                .splineTo(new Vector2d(-36, 60), Math.toRadians(0));

        TrajectoryActionBuilder myTraj2 = drive.actionBuilder(localizer.getPose())
                .splineTo(new Vector2d(60, 60), Math.toRadians(0))
                .splineTo(new Vector2d(20, 20), Math.toRadians(-135))
                .splineTo(new Vector2d(60, -60), Math.toRadians(0));

        TrajectoryActionBuilder Hourglass = drive.actionBuilder(localizer.getPose())
                .splineTo(new Vector2d(36, 60), Math.toRadians(0))
                .splineTo(new Vector2d(0, 0), Math.toRadians(-135))
                .splineTo(new Vector2d(-36, -60), Math.toRadians(0))
                .splineTo(new Vector2d(36, -60), Math.toRadians(0))
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .splineTo(new Vector2d(-36, 60), Math.toRadians(0));


        Action trajectory, trajectory2, trajectory3; ; //defines trajectories as actions
        trajectory = myTraj.build(); //builds trajectories during init
        trajectory2 = myTraj2.build();
        trajectory3 = Hourglass.build();

        waitForStart();
        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectory3
                    ));
        }
    }
}
