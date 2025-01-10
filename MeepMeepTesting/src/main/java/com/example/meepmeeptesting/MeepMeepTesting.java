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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(5, -63, Math.toRadians(90)))
                //deposit preload
                .lineToY(-57)

                //give first sample to human
                .splineTo(new Vector2d(33.6,-32.6),Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(44,-14),Math.toRadians(90))
                .lineToY(-55).setTangent(Math.toRadians(90))

                //give second sample to human

                        .splineToSplineHeading(new Pose2d(48,-14,Math.toRadians(-90)),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(51,-55),Math.toRadians(-90))

                //give third sample to human

                .splineToConstantHeading(new Vector2d(60,-15),Math.toRadians(-90))
                                .lineToY(-55)



                //intake second speciman

                //deposit second speciman

               .strafeToLinearHeading(new Vector2d(5,-37),Math.toRadians(-90))


                //pick up third speciman on floor
                .strafeToLinearHeading(new Vector2d(16,-46.5),Math.toRadians(-45))

                //deposit third speciman
                .strafeToLinearHeading(new Vector2d(5,-37),Math.toRadians(-90))

                //pick up fourth speciman on floor
                .strafeToLinearHeading(new Vector2d(16,-46.5),Math.toRadians(-45))

                //deposit fourth speciman
                .strafeToLinearHeading(new Vector2d(5,-37),Math.toRadians(-90))

                //pick up fifth speciman
                .strafeToLinearHeading(new Vector2d(16,-46.5),Math.toRadians(-45))

                //deposit fifth speciman
                .strafeToLinearHeading(new Vector2d(5,-37),Math.toRadians(-90))

                .strafeToLinearHeading(new Vector2d(28.2,-54),Math.toRadians(-45))


                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}