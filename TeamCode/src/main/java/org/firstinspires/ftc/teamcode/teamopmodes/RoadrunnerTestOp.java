/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
