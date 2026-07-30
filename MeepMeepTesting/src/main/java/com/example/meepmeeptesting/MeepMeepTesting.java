package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static double goToNearest90(double currentHeading) {
//        double currentHeading = getCurrentPose().heading.toDouble();

        double normalize = currentHeading % 90;
        double angleChange = 0;

        if (normalize < 45) {
            angleChange = -normalize;


        } else{
            angleChange = 90 - normalize;
        }

//        com.acmerobotics.roadrunner.ftc.Actions.runBlocking(getActionBuilder().setTangent(0)
//                .turnTo(currentHeading + angleChange)
//                .build());

        return currentHeading + angleChange;
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        // blue lower three row
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .turnTo(Math.toRadians(1))
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(goToNearest90(1)))
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(46))
                .turnTo(Math.toRadians(goToNearest90(46)))
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(359))
                .turnTo(Math.toRadians(goToNearest90(359)))
                .waitSeconds(0.5)

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}