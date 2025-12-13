package org.firstinspires.ftc.teamcode.evergreendynamics.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;

@Autonomous (group = "Evergreen Autos")
public class RedLowerLowRow extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override

    public void runOpMode() {
        try {
            telemetry.addLine("Running Op Mode");
            Pose2d beginPose = new Pose2d(62, 12, Math.toRadians(90));
            float turretStartHeading = 90;
            double robotStartHeading = Math.toDegrees(beginPose.heading.toDouble());

            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.RED_GOAL_POSITION, beginPose);

            waitForStart();

            douglasFIRST.start(robotStartHeading, turretStartHeading);

            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .waitSeconds(2)
                    .build());
            douglasFIRST.shootArtifacts();

            //Moves to upper launch zone
            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .strafeTo(new Vector2d(36, 12))
                    .strafeTo(new Vector2d(36, 62))
                    .build());
            douglasFIRST.savePose();

            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    .strafeTo(new Vector2d(62,12))

                    .waitSeconds(2)
                    .build());
            douglasFIRST.savePose();

            douglasFIRST.shootArtifacts();

            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    .strafeTo(new Vector2d(30,12))
                    .build());

            douglasFIRST.savePose();

        } catch(Exception e) {
            e.printStackTrace();
        } finally {
            douglasFIRST.shutdown();
        }
    }

}