package org.firstinspires.ftc.teamcode.evergreendynamics.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@TeleOp (group = "Evergreen Testing")
public class FlywheelVelocityTest extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override
    public void runOpMode() {
        try {
            Pose2d startPose = new Pose2d (0, 0, 0);
            double turretStartHeading = Math.toDegrees(startPose.heading.toDouble()); //PoseStorage.loadTurretHeading(hardwareMap.appContext);

            telemetry.addLine("The flywheel velocity changes in increments of 5. To increase, press up on the dpad. To decrease, press down on the dpad.");

            telemetry.update();
            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, InputValues.BLUE_GOAL_POSITION, startPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.MANUAL);

            waitForStart();
            douglasFIRST.start(0, 0);

            // Sets up the driving system
            while (opModeIsActive()) {
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
