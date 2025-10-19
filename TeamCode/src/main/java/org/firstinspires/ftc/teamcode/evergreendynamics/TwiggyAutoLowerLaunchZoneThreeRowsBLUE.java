package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous
public class TwiggyAutoLowerLaunchZoneThreeRowsBLUE extends LinearOpMode {
    public Sorter sorter;
    public Intake intake;
    public Turret turret;
    public MecanumDrive mecanumDrive;
    public int aimAtTagId = 20; //Aim at blue goal

    @Override

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Running Op Mode");
        Pose2d beginPose = new Pose2d(0, 0, 0);
        this.mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        this.sorter = new Sorter(hardwareMap, telemetry, gamepad1);
        this.intake = new Intake(hardwareMap, gamepad1);
        this.turret = new Turret(hardwareMap, telemetry, gamepad1, aimAtTagId);

        telemetry.update();

        //Detect motif for artifact order (Init)
        int motifTagId = turret.determineMotif();
        telemetry.addData("Motif Id", motifTagId);
        telemetry.update();

        waitForStart();

        // Intake motor starts, flywheel starts, turret starts looking for the BLUE goal
        turret.backgroundThread.start();
        intake.startSpin();
        turret.startFlywheel();


        //Moves to get first set of three artifacts from the side of the set
        Vector2d strafeVector = new Vector2d(24, 12);
        Actions.runBlocking(
                mecanumDrive.actionBuilder(beginPose).setTangent(0)
                        .strafeTo(new Vector2d(24, 24))
                        .strafeTo(new Vector2d(48, 24))
                        .strafeTo(new Vector2d(72, 0))
                        .build());

        //Waits for artifacts to get into divots, goes through detecting, sorting, flicking
        sleep(2000);
        sorter.detect();

        //Moves to upper launch zone
        mecanumDrive.actionBuilder(beginPose).setTangent(0)
                //.splineToConstantHeading(new Vector2d(24, 24),  Math.toRadians(180))
                //  .splineToConstantHeading(new Vector2d())
                .strafeTo(new Vector2d(24, 24))
                .strafeTo(new Vector2d(48, 24))
                .strafeTo(new Vector2d(72, 0))
                .build();

        //Flicks first artifact
        if (motifTagId == 21) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 22) || (motifTagId == 23)) {
            sorter.flickArtifactPurple();
        }

        //keeps detecting, waits for piston to finish traveling
        sorter.detect();
        sleep(3000);
        sorter.detect();

        //Shoots first artifact
        //turret.shootArtifact();
        sleep(5000);

        //Flick second artifact
        sorter.detect();

        if (motifTagId == 22) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 21) || (motifTagId == 23)) {
            sorter.flickArtifactPurple();
        }

        //keeps detecting, waits for piston to finish traveling
        sorter.detect();
        sleep(3000);
        sorter.detect();

        //Shoots second artifact
        //turret.shootArtifact();

        //Flick third artifact
        sorter.detect();

        if (motifTagId == 23) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 21) || (motifTagId == 22)) {
            sorter.flickArtifactPurple();
        }

        //Shoots third artifact
        //turret.shootArtifact();

        //keeps detecting, waits for piston to finish traveling
        sorter.detect();
        sleep(3000);
        sorter.detect();


        sleep(10000);

    }
}