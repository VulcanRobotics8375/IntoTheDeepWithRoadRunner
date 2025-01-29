package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SampleAutoTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 11.3)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-41, -63, Math.toRadians(180)))


                //deposit preload
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55,-42.7,Math.toRadians(245)),Math.toRadians(135))

                        .turnTo(Math.toRadians(130))
                        .turnTo(Math.toRadians(245))
                .turnTo(Math.toRadians(100))
                .turnTo(Math.toRadians(245))
                .turnTo(Math.toRadians(70))
                .turnTo(Math.toRadians(245))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-22,-8,Math.toRadians(0)),Math.toRadians(0))



                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}