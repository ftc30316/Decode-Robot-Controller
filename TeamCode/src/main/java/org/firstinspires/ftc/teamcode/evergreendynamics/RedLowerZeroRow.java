package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class RedLowerZeroRow extends LinearOpMode {
    public Sorter sorter;
    public Intake intake;
    public Turret turret;
    public MecanumDrive mecanumDrive;

    @Override

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Running Op Mode");
        Pose2d beginPose = new Pose2d(62, 12, Math.toRadians(90));
        this.mecanumDrive = new MecanumDrive(hardwareMap, beginPose);
        this.sorter = new Sorter(hardwareMap, telemetry, gamepad1, gamepad2);
        this.intake = new Intake(hardwareMap, gamepad1, gamepad2, telemetry);
        this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, InputValues.RED_GOAL_POSITION, mecanumDrive);

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
                .strafeToLinearHeading(new Vector2d(50, 10), Math.toRadians(0))
                .build());

        //Detect motif for artifact order (Init)
        int motifTagId = turret.determineMotif();

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