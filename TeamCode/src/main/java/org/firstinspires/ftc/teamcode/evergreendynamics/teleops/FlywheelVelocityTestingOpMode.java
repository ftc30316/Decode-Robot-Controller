package org.firstinspires.ftc.teamcode.evergreendynamics.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Intake;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@TeleOp (group = "Evergreen Testing")
public class FlywheelVelocityTestingOpMode extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override
    public void runOpMode() {
        try {
            Pose2d startPose = new Pose2d (0, 0, 0);
            double turretStartHeading = Math.toDegrees(startPose.heading.toDouble()); //PoseStorage.loadTurretHeading(hardwareMap.appContext);

            telemetry.addData("Flywheel velocity", InputValues.FLYWHEEL_TEST_VELOCITY);

            telemetry.update();
            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, startPose);

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
