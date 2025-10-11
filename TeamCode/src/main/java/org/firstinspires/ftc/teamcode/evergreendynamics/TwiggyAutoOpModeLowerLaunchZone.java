package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous
public class TwiggyAutoOpModeLowerLaunchZone extends LinearOpMode {
    public Sorter sorter;
    public Intake intake;
    public Turret turret;
    public MecanumDrive mecanumDrive;
    public int aimAtTagId = 20; //Aim at blue goal

    @Override

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Running Op Mode");
        this.mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        this.sorter = new Sorter(hardwareMap, telemetry, gamepad1);
        this.intake = new Intake(hardwareMap, gamepad1);
        this.turret = new Turret(hardwareMap, telemetry, gamepad1);

        telemetry.update();

        int tagId = turret.determineMotif();
        telemetry.addData("Motif Id", tagId);
        telemetry.update();

        waitForStart();

        //Reset servos to 0, setup camera to face obelisk direction (Before match separate op mode)
        //Detect motif for artifact order (Init)
        //Moves to get first set of three artifacts from the side of the set
        //Goes through detecting, sorting, flicking
        //Moves to upper launch zone
        //Moves artifacts into flywheels for shooting
        //Turret correction
        //Repeats steps 3-7 for the next two rows of artifacts

        while (opModeIsActive()) {

        }
    }
}