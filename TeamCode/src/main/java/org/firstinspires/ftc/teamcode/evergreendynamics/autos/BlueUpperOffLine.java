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
public class BlueUpperOffLine extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override

    public void runOpMode() {
        try {
        telemetry.addLine("Running Op Mode");
        Pose2d beginPose = new Pose2d(-52, -52, Math.toRadians(45));
        float turretStartHeading = 45;
        double robotStartHeading = Math.toDegrees(beginPose.heading.toDouble());

        this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.BLUE_GOAL_POSITION, beginPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.AUTO);

        waitForStart();

        douglasFIRST.start(robotStartHeading, turretStartHeading);

        //Moves to upper launch zone
        Actions.runBlocking(douglasFIRST.getActionBuilder(beginPose)
                .strafeTo(new Vector2d(-30, -52))
                .build());
            douglasFIRST.savePose();

    } catch(Exception e) {
            e.printStackTrace();
        } finally {

        }
    }

}