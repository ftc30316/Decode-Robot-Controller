package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class TwiggySetupBeforeMatch extends LinearOpMode {
    public Sorter sorter;
    @Override
    public void runOpMode() throws InterruptedException {
        //Reset servos to 0, setup camera to face obelisk direction (Before match separate op mode)
        this.sorter = new Sorter(hardwareMap, telemetry, gamepad1);

        waitForStart();
    }
}