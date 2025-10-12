package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous
public class TwiggyAutoOpModeLowerLaunchZone extends LinearOpMode {
    //public Sorter sorter;
    public Intake intake;
    public Turret turret;
    public MecanumDrive mecanumDrive;
    public int aimAtTagId = 20; //Aim at blue goal

    @Override

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Running Op Mode");
        Pose2d beginPose = new Pose2d(0, 0, 0);
        this.mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //this.sorter = new Sorter(hardwareMap, telemetry, gamepad1);
        this.intake = new Intake(hardwareMap, gamepad1);
        this.turret = new Turret(hardwareMap, telemetry, gamepad1, aimAtTagId);

        telemetry.update();

        //Detect motif for artifact order (Init)
        int tagId = turret.determineMotif();
        telemetry.addData("Motif Id", tagId);
        telemetry.update();

        waitForStart();

        turret.backgroundThread.start();

        //Moves to get first set of three artifacts from the side of the set
        Vector2d strafeVector = new Vector2d(24, 12);
        Actions.runBlocking(
                //drive.actionBuilder(beginPose).splineTo(new Vector2d(24, 24), Math.PI / 2)
                mecanumDrive.actionBuilder(beginPose).setTangent(0)
                        //.splineToConstantHeading(new Vector2d(24, 24),  Math.toRadians(180))
                        //  .splineToConstantHeading(new Vector2d())
                        .strafeTo(new Vector2d(24, 24))
                        .strafeTo(new Vector2d(48, 24))
                        .strafeTo(new Vector2d(72, 0))
                        .build());

        //Goes through detecting, sorting, flicking
        //sorter.detect();

        //Moves to upper launch zone
        mecanumDrive.actionBuilder(beginPose).setTangent(0)
                //.splineToConstantHeading(new Vector2d(24, 24),  Math.toRadians(180))
                //  .splineToConstantHeading(new Vector2d())
                .strafeTo(new Vector2d(24, 24))
                .strafeTo(new Vector2d(48, 24))
                .strafeTo(new Vector2d(72, 0))
                .build();

        //Moves artifacts into flywheels for shooting
        turret.aim(aimAtTagId);
        //Shoots first artifact
        //turret.score();
        //Flick second artifact
        //sorter.detect();
        //Shoots second artifact
        //turret.score();
        //Flick third artifact
        //sorter.detect();
        //Shoots third artifact
        //turret.score();

        //Repeats steps 3-7 for the next two rows of artifacts

        sleep(10000);

    }
}