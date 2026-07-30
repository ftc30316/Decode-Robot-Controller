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
public class TurnTo90Test extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override

    public void runOpMode() {

        try {
            telemetry.addLine("Running Op Mode");
            Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0)); // changed from 270 to -90
            float turretStartHeading = -90;
            double robotStartHeading = Math.toDegrees(beginPose.heading.toDouble());

            waitForStart();

            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.Alliance.BLUE, beginPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.AUTO);



            //Creates background thread
            douglasFIRST.start(robotStartHeading, turretStartHeading, false);

            //
            Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose).setTangent(0)
                            .turnTo(Math.toRadians(167))
                            .waitSeconds(1)
                            .turnTo(Math.toRadians(douglasFIRST.goToNearest90(167)))
                            .waitSeconds(1)
                            .turnTo(Math.toRadians(46))
                            .turnTo(Math.toRadians(douglasFIRST.goToNearest90(46)))
                            .waitSeconds(1)
                            .turnTo(Math.toRadians(359))
                            .turnTo(Math.toRadians(douglasFIRST.goToNearest90(359)))
                            .waitSeconds(1)
                            .build());

            //sleep(5000);

            douglasFIRST.savePose();

        } catch(Exception e) {
            e.printStackTrace();
        } finally {
            //douglasFIRST.turret.stopTurretBackgroundThread();
        }
    }

}