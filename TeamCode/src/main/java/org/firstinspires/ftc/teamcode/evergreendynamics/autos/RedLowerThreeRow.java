package org.firstinspires.ftc.teamcode.evergreendynamics.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Helper;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@Autonomous (group = "Evergreen Autos")
public class RedLowerThreeRow extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override

    public void runOpMode() {
        try {
            telemetry.addLine("Running Op Mode");
            Pose2d beginPose = new Pose2d(63.5, -12, Math.toRadians(-90));
            float turretStartHeading = -90;
            double robotStartHeading = Math.toDegrees(beginPose.heading.toDouble());

            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.Alliance.RED, beginPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.AUTO);

            waitForStart();

            //Creates background thread
            douglasFIRST.start(robotStartHeading, turretStartHeading, true);

            // Moves a little closer to goal
            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .strafeTo(new Vector2d(48, -12))
                    .build());

            // Waits for flywheels to speed up
            Helper.sleep(3000);

            // Shoots preloaded
            douglasFIRST.shootArtifacts();

            // Collects low row and goes back to lower launch zone
            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    // Lines up with low row
                    .strafeTo(new Vector2d(35, -30))
                    // Collects low row
                    .strafeTo(new Vector2d(35,-44))
                    // Goes to lower launch zone to shoot
                    .strafeTo(new Vector2d(48,-12))
                    .build());

            douglasFIRST.savePose();

            // Waits for flywheels to speed up
            Helper.sleep(2000);

            // Shoots low row
            douglasFIRST.shootArtifacts();

            // Collects middle row and goes to upper launch zone
            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    // Lines up with middle row
                    .strafeTo(new Vector2d(11, -30))
                    // Collects middle row
                    .strafeTo(new Vector2d(11, -50))
                    // Backs up
                    //.strafeTo(new Vector2d(11, -55))
                    // Goes to upper launch zone to shoot
                    .strafeTo(new Vector2d(-12, -12))
                    .build());

            douglasFIRST.savePose();

            // Shoots middle row
            douglasFIRST.shootArtifacts();

            // Collects top row and goes back to upper launch zone
            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    // Lines up with upper row
                    .strafeTo(new Vector2d(-13, -30))
                    // Collects upper row
                    .strafeTo(new Vector2d(-13, -50))
                    // Goes to upper launch zone to shoot
                    .strafeTo(new Vector2d(-13, -13))
                    .build());

            douglasFIRST.savePose();

            // Shoots top row
            douglasFIRST.shootArtifacts();

            // Moves off line
            Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                    .strafeTo(new Vector2d(-12, -36))
                    .build());

            douglasFIRST.savePose();

        } catch(Exception e) {
            e.printStackTrace();
        } finally {
            douglasFIRST.turret.stopTurretBackgroundThread();
        }
    }

}