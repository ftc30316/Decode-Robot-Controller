package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class TwiggyTeleOpRED extends LinearOpMode{
    public Sorter sorter;
    public Intake intake;
    public Turret turret;
    public MecanumDrive mecanumDrive;
    public int aimAtTagId = 24; //Aim at red goal

    @Override
    public void runOpMode() throws InterruptedException {
        //this.mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        this.sorter = new Sorter(hardwareMap, telemetry, gamepad1);
        this.intake = new Intake(hardwareMap);
        this.turret = new Turret(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            //intake.spin();
            sorter.detect();
            turret.aim(aimAtTagId);
            telemetry.update();
        }
    }

}
