package org.firstinspires.ftc.teamcode.evergreendynamics.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Intake;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@Disabled

@TeleOp (group = "Evergreen Testing")
public class FlywheelVelocityTestingOpMode extends LinearOpMode {
    public Turret turret;
    public MecanumDrive mecanumDrive;

    @Override
    public void runOpMode() {
        try {
            Pose2d startPose = new Pose2d (0, 0, 0);
            double turretStartHeading = Math.toDegrees(startPose.heading.toDouble()); //PoseStorage.loadTurretHeading(hardwareMap.appContext);

            telemetry.update();
            this.mecanumDrive = new MecanumDrive(hardwareMap, gamepad1, startPose);
            this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, InputValues.BLUE_GOAL_POSITION, mecanumDrive, null);

            waitForStart();

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

                turret.loop();

                telemetry.update();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {

        }

    }
}
