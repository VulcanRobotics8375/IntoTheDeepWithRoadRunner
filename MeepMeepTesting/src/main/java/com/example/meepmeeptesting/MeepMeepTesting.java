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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(5, -63, Math.toRadians(-90)))
                //deposit preload
                .splineToConstantHeading(new Vector2d(5,-35),Math.toRadians(-35))
                .splineToConstantHeading(new Vector2d(35.7,-27.2),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35.7,-16),Math.toRadians(90))

              .splineToConstantHeading(new Vector2d(46.9,-16),Math.toRadians(-90))


                //give first sample to human
                .splineToConstantHeading(new Vector2d(46.9,-52),Math.toRadians(90))

                //give second sample to human
                .splineToConstantHeading(new Vector2d(54,-16),Math.toRadians(-25))
                .splineToConstantHeading(new Vector2d(57,-25),Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(57,-52),Math.toRadians(-90))

                //give third sample to human

                .splineToConstantHeading(new Vector2d(57,-16),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(61,-25),Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(54,-52),Math.toRadians(-180))



                //intake second speciman

                //done on the way

                //deposit second speciman
                .splineToConstantHeading(new Vector2d(3,-35),Math.toRadians(90))


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