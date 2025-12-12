package org.firstinspires.ftc.teamcode.evergreendynamics.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.PoseStorage;

@TeleOp (group = "Evergreen Teleop")
public class RedDouglasTeleOp extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override
    public void runOpMode() {
        try {
            Pose2d startPose = PoseStorage.loadPose(hardwareMap.appContext);
            double turretStartHeadingDeg = PoseStorage.loadTurretHeading(hardwareMap.appContext);
            double robotHeadingDeg = Math.toDegrees(startPose.heading.toDouble());


            telemetry.addData("auto end pose x", startPose.position.x);
            telemetry.addData("auto end pose y", startPose.position.y);
            telemetry.addData("auto end pose heading", Math.toDegrees(startPose.heading.toDouble()));
            telemetry.addData("auto end turret heading", turretStartHeadingDeg);

            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.RED_GOAL_POSITION, startPose); //startPose);

            waitForStart();

            douglasFIRST.start(robotHeadingDeg, turretStartHeadingDeg);

            // Sets up the driving system
            while (opModeIsActive()) {
                telemetry.clearAll();
                douglasFIRST.loop();

                telemetry.update();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            douglasFIRST.shutdown();
        }

    }
}
