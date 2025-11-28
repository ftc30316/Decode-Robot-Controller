package org.firstinspires.ftc.teamcode.evergreendynamics.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Intake;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@Autonomous (group = "Evergreen Autos")
public class RedUpperOffLine extends LinearOpMode {
    public Intake intake;
    public Turret turret;
    public MecanumDrive mecanumDrive;

    @Override

    public void runOpMode() {
        try {
            telemetry.addLine("Running Op Mode");
            Pose2d beginPose = new Pose2d(-50, 50, Math.toRadians(-45));
            float turretStartHeading = -90;
            this.mecanumDrive = new MecanumDrive(hardwareMap, gamepad1, beginPose);
            this.intake = new Intake(hardwareMap, gamepad1, gamepad2, telemetry);
            this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, InputValues.RED_GOAL_POSITION, mecanumDrive);

            telemetry.update();

            waitForStart();

            //Creates background thread
            turret.createTurretBackgroundThread();
            // Intake motor starts, flywheel starts, turret starts looking for the BLUE goal
            turret.turretBackgroundThread.start();
            //intake.startSpin();

            //Moves to upper launch zone
            Actions.runBlocking(mecanumDrive.actionBuilder(beginPose).setTangent(0)
                    .strafeTo(new Vector2d(-30, 50))//, Math.toRadians(90))
                    .build());
            mecanumDrive.updatePoseEstimate();
            PoseStorage.savePose(hardwareMap.appContext, mecanumDrive.localizer.getPose(), turret.getTurretDegrees());
//
//            // Flicks and shoots the preset artifacts and does backup flicks
//            shootThreeArtifacts();
//
//            mecanumDrive.updatePoseEstimate();
//            Actions.runBlocking(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
//                    .strafeTo(new Vector2d(-9,45))
//                    .strafeTo(new Vector2d(-9,22))
//                    .build());
//            mecanumDrive.updatePoseEstimate();
//            PoseStorage.savePose(hardwareMap.appContext, mecanumDrive.localizer.getPose(), turret.getTurretDegrees());
//
//            // Flicks and shoots the first row artifacts and does backup flicks
//            shootThreeArtifacts();

            turret.stopTurretBackgroundThread();
            sleep(100);
            turret.resetTurretToZero();

//            mecanumDrive.updatePoseEstimate();
//            Actions.runBlocking(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose()).setTangent(0)
//                    .strafeTo(new Vector2d(12, 12))
//                    .strafeTo(new Vector2d(12, 50))
//                    .strafeTo(new Vector2d(-12, 12))
//                    .build());
//            mecanumDrive.updatePoseEstimate();
//            PoseStorage.savePose(hardwareMap.appContext, mecanumDrive.localizer.getPose(), turret.getTurretDegrees());
//
//            // Flicks and shoots the second row artifacts and does backup flicks
//            shootThreeArtifacts();
//
//            mecanumDrive.updatePoseEstimate();
//            Actions.runBlocking(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose()).setTangent(0)
//                    .strafeTo(new Vector2d(36, 12))
//                    .strafeTo(new Vector2d(36, 50))
//                    .strafeTo(new Vector2d(-12, 12))
//                    .build());
//            mecanumDrive.updatePoseEstimate();
//            PoseStorage.savePose(hardwareMap.appContext, mecanumDrive.localizer.getPose(), turret.getTurretDegrees());
//
//            // Flicks and shoots the third row artifacts and does backup flicks
//            shootThreeArtifacts();

            sleep(30000);
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            turret.stopTurretBackgroundThread();
        }
    }

    public void shootThreeArtifacts() {
        // Flicks first artifact
        turret.shootArtifact();

        //Flick second artifact
        turret.shootArtifact();

        //Flick third artifact
        turret.shootArtifact();
    }

}