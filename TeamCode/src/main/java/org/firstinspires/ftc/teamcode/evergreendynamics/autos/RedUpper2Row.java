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
public class RedUpper2Row extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override

    public void runOpMode() {
        try {
            telemetry.addLine("Running Op Mode");
            Pose2d beginPose = new Pose2d(-52, 52, Math.toRadians(-45));
            float turretStartHeading = -45;
            double robotStartHeading = Math.toDegrees(beginPose.heading.toDouble());

            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.Alliance.RED, beginPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.AUTO);

            waitForStart();

            // Creates turret background thread
            douglasFIRST.start(robotStartHeading, turretStartHeading, true);

            //Moves to upper launch zone
            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .strafeToLinearHeading(new Vector2d(-12, 25), Math.toRadians(90))
                    .waitSeconds(1)
                    .build());
            douglasFIRST.savePose();

            // Shoots preloaded artifacts
            douglasFIRST.shootArtifacts();

            // Collects upper row
            Actions.runBlocking(douglasFIRST.getActionBuilder()
                    .strafeTo(new Vector2d(-12,60), new TranslationalVelConstraint(20.0))
                    .strafeTo(new Vector2d(-12,25))
                    .waitSeconds(1)
                    .build());
            douglasFIRST.savePose();

            // Shoots upper row artifacts
            douglasFIRST.shootArtifacts();

            Actions.runBlocking(douglasFIRST.getActionBuilder()
                    .strafeTo(new Vector2d(13,25), new TranslationalVelConstraint(15.0))
                    .strafeTo(new Vector2d(13,60))
                    .build());
            douglasFIRST.savePose();

            Actions.runBlocking(douglasFIRST.getActionBuilder()
                    .strafeTo(new Vector2d(-48,24), new TranslationalVelConstraint(15.0))
                    .waitSeconds(1)
                    .build());

            douglasFIRST.shootArtifacts();

            douglasFIRST.savePose();

        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            douglasFIRST.turret.stopTurretBackgroundThread();
        }
    }

}