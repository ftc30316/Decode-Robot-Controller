package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class TwiggyTeleOpRED extends LinearOpMode{
    public Sorter sorter;
    public Intake intake;
    public Turret turret;
    public MecanumDrive mecanumDrive;
    public int aimAtTagId = 24; //Aim at RED goal

    @Override
    public void runOpMode() throws InterruptedException {
        this.mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        this.sorter = new Sorter(hardwareMap, telemetry, gamepad1, gamepad2);
        this.intake = new Intake(hardwareMap, gamepad1, telemetry);
        this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, aimAtTagId);

        waitForStart();

        // Starts the auto-lock on the RED aprilTag
        turret.backgroundThread.start();

        // Flywheel and intake motor start
        turret.startFlywheel();
        intake.startSpin();

        // Sets up the driving system
        while (opModeIsActive()) {
            mecanumDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            sorter.detect();
            turret.score();
            telemetry.update();
        }
    }

}
