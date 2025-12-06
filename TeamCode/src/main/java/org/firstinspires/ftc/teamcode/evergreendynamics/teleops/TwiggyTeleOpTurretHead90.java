package org.firstinspires.ftc.teamcode.evergreendynamics.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@TeleOp (group = "Evergreen Testing")
@Disabled

public class TwiggyTeleOpTurretHead90 extends LinearOpMode {

    public Turret turret;
    public MecanumDrive mecanumDrive;

    @Override
    public void runOpMode() {
        try {
            // Your robot's starting heading and turret starting field angle:
            double robotHeadingStartDeg = 0.0;         // e.g., facing +X at start
            double turretFieldAngleStartDeg = 90;     // turret initially pointing +Y in field coords

            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(robotHeadingStartDeg));

            telemetry.addData("start x", startPose.position.x);
            telemetry.addData("start y", startPose.position.y);
            telemetry.addData("start heading", Math.toDegrees(startPose.heading.toDouble()));
            telemetry.addData("start turret heading", turretFieldAngleStartDeg);

            telemetry.update();

            this.mecanumDrive = new MecanumDrive(hardwareMap, gamepad1, startPose);
//            this.turret = new TurretMathUpgrade(hardwareMap, telemetry, gamepad1, gamepad2,
//                    InputValues.RED_GOAL_POSITION, turretFieldAngleStartDeg, mecanumDrive);


            waitForStart();

//            turret.initialize(robotHeadingStartDeg, turretFieldAngleStartDeg);
//
//
//            //Creates background thread
//            turret.createTurretBackgroundThread();
//            // Intake motor starts, flywheel starts, turret starts looking for the BLUE goal
//            turret.turretBackgroundThread.start();

            // Sets up the driving system
            while (opModeIsActive()) {
                telemetry.addData("robot speed", "FAST");
                mecanumDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                telemetry.update();
//                idle();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            //turret.stopTurretBackgroundThread();
        }

    }
}
