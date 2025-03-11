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

package org.firstinspires.ftc.teamcode.processors.opmodes;

import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.opencv.core.*;
import org.openftc.easyopencv.*;


import java.lang.Math;


@Autonomous(name="Basic: Iterative OpMode", group="Iterative OpMode")
public class AutoS1RR extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        Trajectory trajOne = drive.trajectorySequenceBuilder(new Pose2d(-36, 60, 230))
                .lineTo(new Vector2d(-12,36))
                .build());

        Trajectory trajTwo = drive.trajectorySequenceBuilder(new Pose2d(-12, 36, 190))
                .addDisplacementMarker(() -> {
                    //score specimen
                })
                .build());
        Trajectory trajThree = drive.trajectorySequenceBuilder(new Pose2d(-12, 36, 100))
                .forward(24)
                .build());
        Trajectory trajFour = drive.trajectorySequenceBuilder(newPose2d(-36, 36, 150))
                .addDisplacementMarker(() -> {
                    //lower arm to sweeping position
                })
                .build());
        Trajectory trajFive = drive.trajectorySequenceBuilder(newPose2d(-36, 36, 100))
                .addDisplacementMarker(() -> {
                    //raise arm to upper position
                })
                .forward(8)
                .build());
        Trajectory trajSix = drive.trajectorySequenceBuilder(newPose2d(-44, 36, 150))
                .addDisplacementMarker(() -> {
                    //lower arm to sweeping position
                })
                .build());
        Trajectory trajSeven = drive.trajectorySequenceBuilder(newPose2d(-44, 36, 100))
                .addDisplacementMarker(() -> {
                    //raise arm to upper position
                })
                .forward(8)
                .build());
        Trajectory trajEight = drive.trajectorySequenceBuilder(newPose2d(-52, 36, 150))
                .addDisplacementMarker(() -> {
                    //lower arm to sweeping position
                })
                .build());
        Trajectory trajNine = drive.trajectorySequenceBuilder(newPose2d(-52, 36, 100))
                .addDisplacementMarker(() -> {
                    //raise arm to upper position
                })
                .build());
        Trajectory trajTen = drive.trajectorySequenceBuilder(newPose2d(-52, 36, 130))

                .lineTo(new Vector2d(-12,60))
                .build());
        Trajectory trajEleven = drive.trajectorySequenceBuilder(newPose2d(-12, 60, 100))
                .forward(24)
                .build());
        Trajectory trajTwelve = drive.trajectorySequenceBuilder(newPose2d(-36, 60, 55))
                .lineTo(new Vector2d(-12, 36))
                .build());
        Trajectory trajThirteen = drive.trajectorySequenceBuilder(newPose2d(-12, 60, 55))
                .lineTo(new Vector2d(-36,60))
                .lineTo(new Vector2d(-12, 36))
                .build());
        Trajectory trajFourteen = drive.trajectorySequenceBuilder(newPose2d(-12, 36, -80))
                .lineTo(new Vector2d(-36,60))
                .lineTo(new Vector2d(-12, 36))
                .build());
        Trajectory trajFifteen = drive.trajectorySequenceBuilder(newPose2d(-12, 36, 55))
                .addDisplacementMarker(() -> {
                    //score specimen ( add these for all cycles)
                })
                .build());
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();

        drive.turn(Math.toRadians(-40));
        drive.followTrajectory(trajOne);
        drive.turn(Math.toRadians(-40));
        drive.followTrajectory(trajTwo);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(trajThree);
        drive.turn(Math.toRadians(50));
        drive.followTrajectory(trajFour);
        drive.turn(Math.toRadians(-110));
        drive.turn(Math.toRadians(60));
        drive.followTrajectory(trajFive);
        drive.turn(Math.toRadians(50));
        drive.followTrajectory(trajSix);
        drive.turn(Math.toRadians(-110));
        drive.turn(Math.toRadians(60));
        drive.followTrajectory(trajSeven);
        drive.turn(Math.toRadians(50));
        drive.followTrajectory(trajEight);
        drive.turn(Math.toRadians(-110));
        drive.turn(Math.toRadians(60));
        drive.followTrajectory(trajNine);
        drive.waitSeconds(0.5);
        drive.turn(Math.toRadians(30));
        drive.followTrajectory(trajTen);
        drive.turn(Math.toRadians(-30));
        drive.followTrajectory(trajEleven);
        drive.turn(Math.toRadians(-45));
        drive.followTrajectory(trajTwelve);
        drive.turn(Math.toRadians(135));
        drive.waitSeconds(1);
        drive.turn(Math.toRadians(-135));
        drive.followTrajectory(trajThirteen);
        drive.turn(Math.toRadians(-135));
        drive.followTrajectory(trajFourteen);
        drive.turn(Math.toRadians(135));
        drive.followTrajectory(trajFifteen);

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
