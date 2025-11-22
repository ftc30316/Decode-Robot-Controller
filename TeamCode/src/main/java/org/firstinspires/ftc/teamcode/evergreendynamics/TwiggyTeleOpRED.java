package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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

    @Override
    public void runOpMode() {
        try {
            Pose2d startPose = PoseStorage.loadPose(hardwareMap.appContext);
            double turretStartHeading = Math.toDegrees(startPose.heading.toDouble());
            this.mecanumDrive = new MecanumDrive(hardwareMap, gamepad2, startPose);
            this.sorter = new Sorter(hardwareMap, telemetry, gamepad1, gamepad2);
            this.intake = new Intake(hardwareMap, gamepad1, gamepad2, telemetry);
            this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, InputValues.RED_GOAL_POSITION, turretStartHeading, mecanumDrive);

            waitForStart();

            //Creates background thread
            turret.createTurretBackgroundThread();
            // Intake motor starts, flywheel starts, turret starts looking for the BLUE goal
            turret.turretBackgroundThread.start();

            // Sets up the driving system
            while (opModeIsActive()) {
                if (mecanumDrive.drivePowers == MecanumDrive.DrivePowers.SLOW) {
                    telemetry.addData("robot speed", "SLOW");
                    mecanumDrive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y * 0.25,
                                    -gamepad1.left_stick_x * 0.25
                            ),
                            -gamepad1.right_stick_x * 0.25
                    ));

                } else {
                    telemetry.addData("robot speed", "FAST");
                    mecanumDrive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));
                }

                mecanumDrive.loop();
                sorter.detect();
                turret.score();
                turret.turretControl();

                //Flywheel and intake motor start
                turret.triggerFlywheel();
                intake.triggerIntake();

                // Send all telemetry to driver hub
                telemetry.update();
            }
        } catch(Exception e) {
            // do nothing
        } finally {
            turret.stopTurretBackgroundThread();
        }
    }

}
