package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorEvergreen extends LinearOpMode {
    private DistanceSensor sensor;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "distancesensorPort2");
        waitForStart();

        double range = 10;

        while (opModeIsActive()) {
            double distance = sensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distance of object", distance);

            //says whether something is in range or not
            if (distance > range) {
                telemetry.addData("Something in range?", "No");
                //there is not something within the range
            }
            else {
                telemetry.addData("Something in range", "Yes");
                //There is something within the range
            }

            //control the range
            if (gamepad1.dpad_up) {
                range = range + .1;
            }
            if (gamepad1.dpad_down) {
                range = range -.1;
            }
            telemetry.addData("Range", range);
            telemetry.update();
        }
    }
}
