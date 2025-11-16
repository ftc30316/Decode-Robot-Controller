package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class TwiggyTeleOpBLUE extends LinearOpMode{
    public Sorter sorter;
    public Intake intake;
    public Turret turret;
    public MecanumDrive mecanumDrive;
    public boolean triangleWasDown;
    public boolean fieldCentric;

    @Override
    public void runOpMode() throws InterruptedException {
        this.mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        this.sorter = new Sorter(hardwareMap, telemetry, gamepad1, gamepad2);
        this.intake = new Intake(hardwareMap, gamepad1, gamepad2, telemetry);
        this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, InputValues.BLUE_GOAL_POSITION, mecanumDrive);

        waitForStart();

        // Starts the auto-lock on the BLUE goal
        //turret.turretBackgroundThread.start();

        // Sets up the driving system
        while (opModeIsActive()) {
            if (mecanumDrive.drivePowers == MecanumDrive.DrivePowers.SLOW) {
                mecanumDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * 0.5,
                                -gamepad1.left_stick_x * 0.5
                        ),
                        -gamepad1.right_stick_x * 0.5
                ));

            } else {
                mecanumDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));
            }

//            // Toggle between field-centric and robot-centric with X
//            if (gamepad1.triangle && !triangleWasDown) fieldCentric = !fieldCentric;
//            triangleWasDown = gamepad1.triangle;
//
//            // Get driver inputs
//            double forward = -gamepad1.left_stick_y;  // Forward
//            double strafe  = -gamepad1.left_stick_x;  // Left
//            double turn    = -gamepad1.right_stick_x; // CCW rotation
//
//            // Create desired field-frame motion vector
//            PoseVelocity2d driveInput = new Pose2d(forward, strafe, turn);
//
//            // Convert to robot frame if field-centric
//            if (fieldCentric) {
//                double heading = mecanumDrive.localizer.getPose().heading.toDouble(); // Radians
//                double cosA = Math.cos(-heading);
//                double sinA = Math.sin(-heading);
//
//                Vector2d robotX = driveInput.position.x * cosA - driveInput.position.y * sinA;
//                Vector2d robotY = driveInput.position.x * sinA + driveInput.position.y * cosA;
//
//                driveInput = new PoseVelocity2d(robotX, robotY, turn);
//            }
//
//            mecanumDrive.setDrivePowers(driveInput);


            sorter.detect();
            turret.score();
            turret.turretControl();

            //Flywheel and intake motor start
            turret.triggerFlywheel();
            intake.triggerIntake();
            turret.adjustTurret();
            telemetry.update();
        }
    }

}
