package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // blue lower launch zone auto
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, -12, Math.toRadians(270)))
//                .strafeTo(new Vector2d(-12, -12))
//                // shoots preloaded set of artifacts
//                .waitSeconds(10)
//                // grabs first set of artifacts
//                .strafeTo(new Vector2d(-12,-50))
//                // goes to shooting position
//                .strafeTo(new Vector2d(-12,-12))
//                // shoots first set
//                .waitSeconds(10)
//                .strafeTo(new Vector2d(12,-12))
//                // grabs second set of artifacts
//                .strafeTo(new Vector2d(12,-50))
//                // goes to shooting position
//                .strafeTo(new Vector2d(-12,-12))
//                // shoots second set of artifacts
//                .waitSeconds(10)
//                .strafeTo(new Vector2d(36,-12))
//                // grabs third set
//                .strafeTo(new Vector2d(36,-50))
//                // goes to shooting position
//                .strafeTo(new Vector2d(-12,-12))
//                // shoots third set
//                .waitSeconds(10)
//
//                .build());

        // blue upper launch zone auto
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))
//                .strafeToLinearHeading(new Vector2d(-12, -12), Math.toRadians(-90))
//                .waitSeconds(10)
//                .strafeTo(new Vector2d(-12,-50))
//                .strafeTo(new Vector2d(-12,-12))
//                .waitSeconds(10)
//                .strafeTo(new Vector2d(12,-12))
//                .strafeTo(new Vector2d(12,-50))
//                .strafeTo(new Vector2d(-12,-12))
//                .waitSeconds(10)
//                .strafeTo(new Vector2d(36,-12))
//                .strafeTo(new Vector2d(36,-50))
//                .strafeTo(new Vector2d(-12,-12))
//                .waitSeconds(10)
//
//                .build());

        // red upper launch zone auto
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-52, 52, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .waitSeconds(10)
                .strafeTo(new Vector2d(-12,50))
                .strafeTo(new Vector2d(-12,12))
                .waitSeconds(10)
                .strafeTo(new Vector2d(12,12))
                .strafeTo(new Vector2d(12,50))
                .strafeTo(new Vector2d(-12,12))
                .waitSeconds(10)
                .strafeTo(new Vector2d(36,12))
                .strafeTo(new Vector2d(36,50))
                .strafeTo(new Vector2d(-12,12))
                .waitSeconds(10)
                .build());

        // red lower launch zone auto
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, 12, Math.toRadians(90)))
                .strafeTo(new Vector2d(-12, 12))
                // shoots preloaded set of artifacts
                .waitSeconds(10)
                // grabs first set of artifacts
                .strafeTo(new Vector2d(-12,50))
                // goes to shooting position
                .strafeTo(new Vector2d(-12,12))
                // shoots first set
                .waitSeconds(10)
                .strafeTo(new Vector2d(12,12))
                // grabs second set of artifacts
                .strafeTo(new Vector2d(12,50))
                // goes to shooting position
                .strafeTo(new Vector2d(-12,12))
                // shoots second set of artifacts
                .waitSeconds(10)
                .strafeTo(new Vector2d(36,12))
                // grabs third set
                .strafeTo(new Vector2d(36,50))
                // goes to shooting position
                .strafeTo(new Vector2d(-12,12))
                // shoots third set
                .waitSeconds(10)

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}