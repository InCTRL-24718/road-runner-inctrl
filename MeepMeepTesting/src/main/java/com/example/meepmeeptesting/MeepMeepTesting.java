package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 24)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, 60, 0))
                        .turn(Math.toRadians(-40))
                        .lineTo(new Vector2d(-12,36))
                        .turn(Math.toRadians(-50))
                        .waitSeconds(2)
                        .turn(Math.toRadians(-90))
                        .forward(24)
                        .waitSeconds(1)
                        .turn(Math.toRadians(-60))
                        .turn(Math.toRadians(60))
                        .forward(8)
                        .turn(Math.toRadians(-90))
                        .turn(Math.toRadians(90))
                        .forward(8)
                        .turn(Math.toRadians(-90))
                        .turn(Math.toRadians(90))
                        .waitSeconds(2)
                        .turn(Math.toRadians(30))
                        .lineTo(new Vector2d(-12,60))
                        .turn(Math.toRadians(120))
                        .forward(60)
                        .turn(Math.toRadians(75))
                        .forward(24)
                        .waitSeconds(3)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}