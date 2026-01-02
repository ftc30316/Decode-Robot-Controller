package org.firstinspires.ftc.teamcode.evergreendynamics.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

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

            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.RED_GOAL_POSITION, beginPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.AUTO);

            waitForStart();

            douglasFIRST.start(robotStartHeading, turretStartHeading);

            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .waitSeconds(2)
                    .build());

            // Shoots preloaded artifacts
            douglasFIRST.shootArtifacts();

            // Collects lower row
            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .strafeTo(new Vector2d(36, 12), new TranslationalVelConstraint(5.0))
                    .strafeTo(new Vector2d(36, 62))
                    .build());
            douglasFIRST.savePose();

            // Moves back to lower launch zone
            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    .strafeTo(new Vector2d(62,12))
                    .build());
            douglasFIRST.savePose();

            // Shoots lower row artifacts
            douglasFIRST.shootArtifacts();

            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    .strafeTo(new Vector2d(30,12))
                    .build());

            douglasFIRST.savePose();

        } catch(Exception e) {
            e.printStackTrace();
        } finally {

        }
    }

}