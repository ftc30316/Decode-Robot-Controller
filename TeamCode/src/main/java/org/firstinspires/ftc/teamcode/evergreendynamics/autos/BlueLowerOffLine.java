package org.firstinspires.ftc.teamcode.evergreendynamics.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Intake;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@Autonomous (group = "Evergreen Autos")
public class BlueLowerOffLine extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override

    public void runOpMode() {
        try {
            telemetry.addLine("Running Op Mode");
            Pose2d beginPose = new Pose2d(62, -12, Math.toRadians(-90)); // changed from 270 to -90
            float turretStartHeading = -90;
            double robotStartHeading = Math.toDegrees(beginPose.heading.toDouble());


            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.BLUE_GOAL_POSITION, beginPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.AUTO);

            waitForStart();

            // Creates background thread
            douglasFIRST.start(robotStartHeading, turretStartHeading, true);

            //Moves off line
            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                    .strafeTo(new Vector2d(30, -12))
                    .build());

            douglasFIRST.savePose();

            sleep(30000);
        } catch(Exception e) {
            e.printStackTrace();
        } finally {
            douglasFIRST.turret.stopTurretBackgroundThread();
        }
    }

}