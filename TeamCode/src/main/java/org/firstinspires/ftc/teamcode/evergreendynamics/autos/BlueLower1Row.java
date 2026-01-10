package org.firstinspires.ftc.teamcode.evergreendynamics.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@Autonomous (group = "Evergreen Autos")
public class BlueLower1Row extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override

    public void runOpMode() {
        try {
            telemetry.addLine("Running Op Mode");
            Pose2d beginPose = new Pose2d(62, -12, Math.toRadians(-90)); // changed from 270 to -90
            float turretStartHeading = -90;
            double robotStartHeading = Math.toDegrees(beginPose.heading.toDouble());

            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.Alliance.BLUE, beginPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.AUTO);

            waitForStart();

            //Creates background thread
            douglasFIRST.start(robotStartHeading, turretStartHeading, true);

            // strafes to upper launch zone
            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .strafeTo(new Vector2d(-12, -20))
                    .build());
            douglasFIRST.savePose();

            // shoots preloaded
            douglasFIRST.shootArtifacts();

            // collects first row of artifacts
            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    .strafeTo(new Vector2d(-12,-52), new TranslationalVelConstraint(5.0))
                    .strafeTo(new Vector2d(-12,-20))
                    .build());
            douglasFIRST.savePose();

            douglasFIRST.shootArtifacts();

            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    .strafeTo(new Vector2d(-12,-50))

                    .build());
            douglasFIRST.savePose();

        } catch(Exception e) {
            e.printStackTrace();
        } finally {
            douglasFIRST.turret.stopTurretBackgroundThread();
        }
    }

}