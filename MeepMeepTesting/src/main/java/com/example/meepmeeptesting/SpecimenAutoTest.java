package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SpecimenAutoTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 11.3)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, -59, Math.toRadians(90)))

                // preload depo spec

                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(8,-38),Math.toRadians(90))





                //give first sample to human

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(39,-45,Math.toRadians(70)),Math.toRadians(0))

                .waitSeconds(0.5)
                .turnTo(Math.toRadians(-60))



                //give second sample to human
                .setTangent(Math.toRadians(20))
                .splineToLinearHeading(new Pose2d(47,-41.5,Math.toRadians(48)),Math.toRadians(0)) // prev math1 52
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(47,-44,Math.toRadians(-90)),Math.toRadians(-90)) // prev y was -42.5



                //pick up second spec: bring lift up, close claw

                //deposit second spec
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(6,-48,Math.toRadians(90)),Math.toRadians(180))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(6,-34),Math.toRadians(90))




                //pick up third spec
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(35,-42,Math.toRadians(-90)),Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(35,-52),Math.toRadians(-90))





                //deposit third spec
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(4,-47,Math.toRadians(90)),Math.toRadians(180))

                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(4,-34),Math.toRadians(90))


                //park
                        .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(23,-60,Math.toRadians(0)),Math.toRadians(0))


                .build());


                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}