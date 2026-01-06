package org.firstinspires.ftc.teamcode.evergreendynamics.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@TeleOp (group = "Evergreen Teleop")
public class DemoTeleOp extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override
    public void runOpMode() {
        try {
            Pose2d startPose = new Pose2d(0, 0, 0); //PoseStorage.loadPose(hardwareMap.appContext);
            double turretStartHeadingDeg = PoseStorage.loadTurretHeading(hardwareMap.appContext);
            double robotHeadingDeg = Math.toDegrees(startPose.heading.toDouble());

            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, startPose, DouglasFIRST.DriveMode.FIELD_CENTRIC, Turret.TurretVelocityMode.AUTO);

            waitForStart();

            douglasFIRST.start(0, 0, false);

            // Sets up the driving system
            while (opModeIsActive()) {
                telemetry.clearAll();
                douglasFIRST.loop();

                telemetry.update();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {

        }

    }
}