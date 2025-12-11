package org.firstinspires.ftc.teamcode.evergreendynamics.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;

@Autonomous (group = "Evergreen Autos")
public class BlueUpper1Row extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override

    public void runOpMode() {
        try {
        telemetry.addLine("Running Op Mode");
        Pose2d beginPose = new Pose2d(-52, -52, Math.toRadians(45));
        float turretStartHeading = -90;
        double robotStartHeading = Math.toDegrees(beginPose.heading.toDouble());

        this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.BLUE_GOAL_POSITION, beginPose);


        waitForStart();
        douglasFIRST.start(robotStartHeading, -90);

        //Moves to upper launch zone
        Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-15, -12), Math.toRadians(-90))
                .build());
        douglasFIRST.savePose();

        douglasFIRST.shootArtifacts();

        Actions.runBlocking(douglasFIRST.getActionBuilder().setTangent(0)
                .strafeTo(new Vector2d(-12,-50))
                .strafeTo(new Vector2d(-9,-17))
                .build());
        douglasFIRST.savePose();

        douglasFIRST.shootArtifacts();

    } catch(Exception e) {
            e.printStackTrace();
        } finally {
            douglasFIRST.shutdown();
        }
    }
}