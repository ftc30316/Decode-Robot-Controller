package org.firstinspires.ftc.teamcode.evergreendynamics.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@Autonomous (group = "Evergreen Autos")
public class RedLowerShootStrafe extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override

    public void runOpMode() {
        try {
            telemetry.addLine("Running Op Mode");
            Pose2d beginPose = new Pose2d(62, 12, Math.toRadians(0));
            float turretStartHeading = 0;
            double robotStartHeading = Math.toDegrees(beginPose.heading.toDouble());

            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.RED_GOAL_POSITION, beginPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.AUTO);

            waitForStart();

            douglasFIRST.start(robotStartHeading, turretStartHeading);

            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(52, 12))
                    .build());
            douglasFIRST.savePose();

            //Waits for artifacts to get into divots, goes through detecting, sorting, flicking
            douglasFIRST.shootArtifacts();

            //Moves to upper launch zone
            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .strafeTo(new Vector2d(52, 56))
                    .build());
            douglasFIRST.savePose();


        } catch(Exception e) {
            e.printStackTrace();
        } finally {
            douglasFIRST.shutdown();
        }
    }
}