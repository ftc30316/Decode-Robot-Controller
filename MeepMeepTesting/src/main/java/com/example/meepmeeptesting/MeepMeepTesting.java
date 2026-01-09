package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // blue lower three row
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, -12, Math.toRadians(-90)))
                .waitSeconds(0.5)
                // Lines up with low row
                .strafeTo(new Vector2d(36, -30))
                // Collects low row with slow speed
                .strafeTo(new Vector2d(36,-55), new TranslationalVelConstraint(5.0))
                // Goes to lower launch zone to shoot
                .strafeTo(new Vector2d(62,-12))

                // Shoots

                // Lines up with middle row
                .strafeTo(new Vector2d(12, -30))
                // Collects middle row with slow speed
                .strafeTo(new Vector2d(12, -55), new TranslationalVelConstraint(5.0))
                // Goes to upper launch zone to shoot
                .strafeTo(new Vector2d(-12, -12))

                // Shoots

                // Lines up with upper row
                .strafeTo(new Vector2d(-12, -30))
                // Collects upper row with slow speed
                .strafeTo(new Vector2d(-12, -55), new TranslationalVelConstraint(5.0))
                // Goes to upper launch zone to shoot
                .strafeTo(new Vector2d(-36, -36))
                // Moves off line
                .strafeTo(new Vector2d(-12, -55))

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}