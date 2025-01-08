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
                .lineToY(-38)

                //give first sample to human
                .strafeToLinearHeading(new Vector2d(25.7,-40),Math.toRadians(30))
                .turnTo(Math.toRadians(-45))

                //give second sample to human
                .strafeToLinearHeading(new Vector2d(35.7,-40),Math.toRadians(30))
                .turnTo(Math.toRadians(-45))

                //give third sample to human
                .strafeToLinearHeading(new Vector2d(44.7,-40),Math.toRadians(30))
                .turnTo(Math.toRadians(-45))

                //pick up second speciman on wall
                .strafeToLinearHeading(new Vector2d(35,-50),Math.toRadians(-90))

                //deposit second speciman
                .strafeTo(new Vector2d(5,-37))

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



                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}