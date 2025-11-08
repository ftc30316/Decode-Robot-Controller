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
        Pose2d beginPose = new Pose2d(62, -12, Math.toRadians(270));
        Pose2d goalPose = new Pose2d(-12, -12, Math.toRadians(270));
        this.mecanumDrive = new MecanumDrive(hardwareMap, beginPose);
        this.sorter = new Sorter(hardwareMap, telemetry, gamepad1, gamepad2);
        this.intake = new Intake(hardwareMap, gamepad1, telemetry);
        this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, aimAtTagId);

        telemetry.update();

        //Detect motif for artifact order (Init)
        int motifTagId = turret.determineMotif();
        telemetry.addData("Motif Id", motifTagId);
        telemetry.update();

        waitForStart();

        // Intake motor starts, flywheel starts, turret starts looking for the BLUE goal
        //turret.backgroundThread.start();
        intake.startSpin();
        turret.startFlywheel();


        //Moves to get first set of three artifacts from the side of the set
//        Actions.runBlocking(
//                mecanumDrive.actionBuilder(beginPose).setTangent(0)
//                        .strafeToConstantHeading(new Vector2d(-48, 24))
//                        // Moves away from wall
//                        .strafeTo(new Vector2d(-42,12))
//                        // Moves towards artifacts in row 1
//                        .strafeTo(new Vector2d(-42, 72))
//                        // Moves away from artifacts in row 1
//                        .strafeTo(new Vector2d(-42, 12))
                        // Moves to launch zone
//                        .strafeTo(new Vector2d(-6,54))
                        //.turn(Math.toRadians(15))
//                        .build());

        //Waits for artifacts to get into divots, goes through detecting, sorting, flicking
        sleep(2000);
        sorter.detect();

//        //Moves to upper launch zone
        Actions.runBlocking(mecanumDrive.actionBuilder(beginPose).setTangent(0)
                .strafeTo(new Vector2d(-12, -12))
                .build());

//
        // changes state to flicking
        if (motifTagId == 21) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 22) || (motifTagId == 23)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        //Flick second artifact
        sorter.detect();

        if (motifTagId == 22) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 21) || (motifTagId == 23)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo


        //Flick third artifact
        sorter.detect();

        if (motifTagId == 23) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 21) || (motifTagId == 22)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        sleep(1000);
        // Safety net, flicks all just in case
        sorter.flickAll();
        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        Actions.runBlocking(mecanumDrive.actionBuilder(goalPose).setTangent(0)
                .strafeTo(new Vector2d(-12,-50))
                .strafeTo(new Vector2d(-12,-12))
                .build());

        // changes state to flicking
        if (motifTagId == 21) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 22) || (motifTagId == 23)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        //Flick second artifact
        sorter.detect();

        if (motifTagId == 22) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 21) || (motifTagId == 23)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo


        //Flick third artifact
        sorter.detect();

        if (motifTagId == 23) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 21) || (motifTagId == 22)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        sleep(1000);
        // Safety net, flicks all just in case
        sorter.flickAll();
        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        Actions.runBlocking(mecanumDrive.actionBuilder(goalPose).setTangent(0)
                .strafeTo(new Vector2d(12,-12))
                .strafeTo(new Vector2d(12,-50))
                .strafeTo(new Vector2d(-12,-12))
                .build());

        // changes state to flicking
        if (motifTagId == 21) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 22) || (motifTagId == 23)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        //Flick second artifact
        sorter.detect();

        if (motifTagId == 22) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 21) || (motifTagId == 23)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo


        //Flick third artifact
        sorter.detect();

        if (motifTagId == 23) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 21) || (motifTagId == 22)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        sleep(1000);
        // Safety net, flicks all just in case
        sorter.flickAll();
        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        Actions.runBlocking(mecanumDrive.actionBuilder(goalPose).setTangent(0)
                .strafeTo(new Vector2d(36,-12))
                .strafeTo(new Vector2d(36,-50))
                .strafeTo(new Vector2d(-12,-12))
                .build());

        // changes state to flicking
        if (motifTagId == 21) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 22) || (motifTagId == 23)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        //Flick second artifact
        sorter.detect();

        if (motifTagId == 22) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 21) || (motifTagId == 23)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo


        //Flick third artifact
        sorter.detect();

        if (motifTagId == 23) {
            sorter.flickArtifactGreen();
        } else if ((motifTagId == 21) || (motifTagId == 22)) {
            sorter.flickArtifactPurple();
        }

        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        sleep(1000);
        // Safety net, flicks all just in case
        sorter.flickAll();
        sorter.detect(); // triggers flick
        sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        sorter.detect(); // triggers reset
        sleep((long) (InputValues.SETTLE_TIME * 1000));

        turret.shootArtifact(); // lifts lift servo
        sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        turret.score(); // resets lift servo

        sleep(30000);

    }
}