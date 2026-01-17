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
public class BlueLowerLowRow extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override

    public void runOpMode() {
        try {
            telemetry.addLine("Running Op Mode");
            Pose2d beginPose = new Pose2d(63.5, -12, Math.toRadians(-90));
            float turretStartHeading = -90;
            double robotStartHeading = Math.toDegrees(beginPose.heading.toDouble());


            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.Alliance.BLUE, beginPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.AUTO);

            waitForStart();

            //Creates background thread
            douglasFIRST.start(robotStartHeading, turretStartHeading, true);

            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .strafeTo(new Vector2d(51, -12))
                    .waitSeconds(3)
                    .build());
            douglasFIRST.shootArtifacts();

            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .strafeTo(new Vector2d(38, -12))
                    .strafeTo(new Vector2d(38, -60), new TranslationalVelConstraint(20.0))
                    .build());
            douglasFIRST.savePose();

            // moves back to lower launch zone
            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    .strafeTo(new Vector2d(51,-12))
                    .build());
            douglasFIRST.savePose();

//            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
//                    .waitSeconds(6)
//                    .build());
            // shoots from lower launch zone
            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    .waitSeconds(3)
                    .build());
            douglasFIRST.shootArtifacts();

            // moves off line
            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    .strafeTo(new Vector2d(30,-12))
                    .build());

            douglasFIRST.savePose();

        } catch(Exception e) {
            e.printStackTrace();
        } finally {
            douglasFIRST.turret.stopTurretBackgroundThread();
        }
    }

}