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
                        //deposit preload

                        .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(8,-33),Math.toRadians(90))

                        .setTangent(Math.toRadians(-35)) .splineToConstantHeading(new Vector2d(35.7,-27.2),Math.toRadians(90))

                        //get to first sample
                        .splineToConstantHeading(new Vector2d(35.7,-16),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(46.9,-16),Math.toRadians(-90))

                        //push first sample
                        .splineToConstantHeading(new Vector2d(46.9,-55),Math.toRadians(-90))
                        .setTangent(Math.toRadians(90))

                        //get to second sample
                        .splineToConstantHeading(new Vector2d(46.9,-19),Math.toRadians(60))
                        .splineToConstantHeading(new Vector2d(57,-16),Math.toRadians(-90))

                        //push second sample
                .splineToConstantHeading(new Vector2d(59,-55.5),Math.toRadians(-90))


                        //deposit second spec
                        .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(5,-33),Math.toRadians(90))


                        //pick up third spec
                        .setTangent(Math.toRadians(-45)).splineToConstantHeading(new Vector2d(34.6 ,-59),Math.toRadians(-45))


                        //deposit third spec
                        .setTangent(Math.toRadians(180)) .splineToLinearHeading(new Pose2d(2,-39,Math.toRadians(90)),Math.toRadians(90))

                        //pick up fourth spec
                        .setTangent(Math.toRadians(-45)).splineToConstantHeading(new Vector2d(34.6 ,-59),Math.toRadians(-45))

                        //deposit fourth spec
                        .setTangent(Math.toRadians(180)) .splineToLinearHeading(new Pose2d(1,-39,Math.toRadians(90)),Math.toRadians(90))

                        //park
                        .setTangent(Math.toRadians(-45)) .splineToLinearHeading(new Pose2d(26,-50,Math.toRadians(-45)),Math.toRadians(-45))


                        .build());


                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}