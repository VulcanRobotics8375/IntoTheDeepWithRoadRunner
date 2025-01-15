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
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 11.3)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(8,-35),Math.toRadians(90))

                //deposit preload
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
                .splineToConstantHeading(new Vector2d(57,-55),Math.toRadians(-90))



                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}