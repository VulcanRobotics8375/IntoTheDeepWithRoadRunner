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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-41, -63, Math.toRadians(0)))

                //preload sample deposit
                .setTangent(Math.toRadians(180))

                .splineToConstantHeading(new Vector2d(-50, -62),Math.toRadians(180))


                //intake sample 2
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-50,-50.5,Math.toRadians(90)),Math.toRadians(90))



                //deposit sample 2
                .turnTo(Math.toRadians(45))

                .splineToConstantHeading(new Vector2d(-55.5, -54), Math.toRadians(45))
                .waitSeconds(1)




                //intake sample 3
                .turnTo(Math.toRadians(92))

                .splineToConstantHeading(new Vector2d(-58, -50.7), Math.toRadians(92))
                .waitSeconds(1)




                //deposit sample 3
                .turnTo(Math.toRadians(45))

                .splineToConstantHeading(new Vector2d(-55.5, -54), Math.toRadians(45))
                .waitSeconds(1)



                //intake sample 4

                .turnTo(Math.toRadians(120))

                .splineToConstantHeading(new Vector2d(-59.4, -49.5), Math.toRadians(120))
                .waitSeconds(1)


                //deposit sample 4
                .turnTo(Math.toRadians(45))

                .splineToConstantHeading(new Vector2d(-55.5, -54), Math.toRadians(45))
                .waitSeconds(1)

                //park or go pick up sample
                .splineTo(new Vector2d(-29,-8.7),Math.toRadians(0))

                //deposit fifth sample?
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-53.5,-53.5,Math.toRadians(45)),Math.toRadians(-90))


                //park or go pick up sample
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-29,-8.7,Math.toRadians(0)),Math.toRadians(0))

                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}