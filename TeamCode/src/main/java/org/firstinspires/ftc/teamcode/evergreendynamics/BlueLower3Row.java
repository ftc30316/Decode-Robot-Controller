package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous
public class BlueLower3Row extends LinearOpMode {
    public Sorter sorter;
    public Intake intake;
    public Turret turret;
    public MecanumDrive mecanumDrive;

    @Override

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Running Op Mode");
        Pose2d beginPose = new Pose2d(62, -12, Math.toRadians(270));
        this.mecanumDrive = new MecanumDrive(hardwareMap, gamepad1, beginPose);
        this.sorter = new Sorter(hardwareMap, telemetry, gamepad1, gamepad2);
        this.intake = new Intake(hardwareMap, gamepad1, gamepad2, telemetry);
        this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, InputValues.BLUE_GOAL_POSITION, mecanumDrive);

        telemetry.update();

        waitForStart();

        //Creates background thread
        turret.createTurretBackgroundThread();
        // Intake motor starts, flywheel starts, turret starts looking for the BLUE goal
        //turret.turretBackgroundThread.start();
        intake.startSpin();
        turret.startFlywheel();

        //Waits for artifacts to get into divots, goes through detecting, sorting, flicking
        sorter.detect();

        //Moves to upper launch zone
        Actions.runBlocking(mecanumDrive.actionBuilder(beginPose).setTangent(0)
                .strafeTo(new Vector2d(-12, -12))
                .build());

        //Detect motif for artifact order (Init)
        int motifTagId = turret.determineMotif();

        // Flicks and shoots the preset artifacts and does backup flicks
        shootThreeArtifacts(motifTagId);

        mecanumDrive.updatePoseEstimate();
        Actions.runBlocking(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose()).setTangent(0)
                .strafeTo(new Vector2d(-12,-50))
                .strafeTo(new Vector2d(-12,-12))
                .build());

        // Flicks and shoots the first row artifacts and does backup flicks
        shootThreeArtifacts(motifTagId);

        mecanumDrive.updatePoseEstimate();
        Actions.runBlocking(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose()).setTangent(0)
                .strafeTo(new Vector2d(12,-12))
                .strafeTo(new Vector2d(12,-50))
                .strafeTo(new Vector2d(-12,-12))
                .build());

        // Flicks and shoots the second row artifacts and does backup flicks
        shootThreeArtifacts(motifTagId);

        mecanumDrive.updatePoseEstimate();
        Actions.runBlocking(mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose()).setTangent(0)
                .strafeTo(new Vector2d(36,-12))
                .strafeTo(new Vector2d(36,-50))
                .strafeTo(new Vector2d(-12,-12))
                .build());

        // Flicks and shoots the third row artifacts and does backup flicks
        shootThreeArtifacts(motifTagId);

        sleep(30000);
    }

    public void shootThreeArtifacts(int motifTagId) {
        // Flicks first artifact
        sorter.flickForMotif(motifTagId, 1);
        turret.shootArtifact();

        //Flick second artifact
        sorter.flickForMotif(motifTagId, 2);
        turret.shootArtifact();

        //Flick third artifact
        sorter.flickForMotif(motifTagId, 3);
        turret.shootArtifact();

        // Safety net, flicks all just in case
        sorter.backupFlickAll();
    }
}