package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class TwiggySetupBeforeMatch extends LinearOpMode {
    public Sorter sorter;
    @Override
    public void runOpMode() throws InterruptedException {
        //Reset servos to 0, manually setup camera to face obelisk direction (This is an op mode to be played before the match)
        this.sorter = new Sorter(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();
    }
}