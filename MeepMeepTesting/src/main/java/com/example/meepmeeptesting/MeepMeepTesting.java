package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.3)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35, -63, Math.toRadians(45)))

                //preload sample deposit
                        .strafeTo(new Vector2d(-53, -56.5))

                .waitSeconds(1)

                //sample 2 intake
                        .strafeTo(new Vector2d(-58, -46.7))
                        .turnTo(Math.toRadians(65))
                        .turnTo(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-54, -54), Math.toRadians(45))

                .turnTo(Math.toRadians(92))
                .splineToConstantHeading(new Vector2d(-60, -46.7), Math.toRadians(92))

                .turnTo(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-54, -54), Math.toRadians(45))

                .turnTo(Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(-60, -46.7), Math.toRadians(110))

                .turnTo(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-54, -54), Math.toRadians(45))

                .turnTo(0)
                .splineToConstantHeading(new Vector2d(-29, -8.7), Math.toRadians(0))

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}