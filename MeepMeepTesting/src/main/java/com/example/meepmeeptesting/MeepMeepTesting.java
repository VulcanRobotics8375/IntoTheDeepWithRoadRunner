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
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 11.3)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-37, -63, Math.toRadians(0)))
                //preload sample deposit

                //preload sample deposit
                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-54, -61),Math.toRadians(180)).waitSeconds(1)

                //intake sample 2
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-50,-49.2,Math.toRadians(90)),Math.toRadians(0))

                //deposit sample 2
                .waitSeconds(1)

                .splineToLinearHeading(new Pose2d(-57,-57,Math.toRadians(45)),Math.toRadians(90))

                .waitSeconds(1)

                //intake sample 3

                .splineToLinearHeading(new Pose2d(-58,-50.7,Math.toRadians(90)),Math.toRadians(90))


                .waitSeconds(2)



                //deposit sample 3


                .splineToLinearHeading(new Pose2d(-57,-57,Math.toRadians(45)),Math.toRadians(90))

                .waitSeconds(1)


                //intake sample 4

                .splineToLinearHeading(new Pose2d(-58.4,-49.5,Math.toRadians(120)),Math.toRadians(90))


                .waitSeconds(1)

                //deposit sample 4

                .splineToLinearHeading(new Pose2d(-57,-57,Math.toRadians(45)),Math.toRadians(90))

                .waitSeconds(1)


                //park

                .splineTo(new Vector2d(-29,-8.7),Math.toRadians(0))

                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}