package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.1)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, 60, 0))
                        .splineTo(new Vector2d(36, 60), Math.toRadians(0))
                        .splineTo(new Vector2d(0, 0), Math.toRadians(-135))
                        .splineTo(new Vector2d(-36, -60), Math.toRadians(0))
                        .splineTo(new Vector2d(36, -60), Math.toRadians(0))
                        .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                        .splineTo(new Vector2d(-36, 60), Math.toRadians(0))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}