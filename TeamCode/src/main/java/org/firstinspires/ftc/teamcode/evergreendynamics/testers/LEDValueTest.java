
// To make LED 1 go through the color cycle, press up on the dpad. Press down to go in reverse.
// To make LED 2 go through the color cycle, press right on the dpad. Press left to go in reverse.

// 0.27 and below is off
// 0.73 and above is white

package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class LEDValueTest extends LinearOpMode {
    private Servo artifactLED;
    private Servo launchZoneLED;
    double artifactLEDValue = 0.25;
    double launchZoneValue = 0.25;

    @Override
    public void runOpMode() {

        artifactLED = hardwareMap.get(Servo.class, "LED1");
        launchZoneLED = hardwareMap.get(Servo.class, "LED2");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpadUpWasPressed()) {
                artifactLEDValue = artifactLEDValue + 0.01;
                artifactLED.setPosition(artifactLEDValue);
            }
            if (gamepad1.dpadDownWasPressed()) {
                artifactLEDValue = artifactLEDValue - 0.01;
                artifactLED.setPosition(artifactLEDValue);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                launchZoneValue = launchZoneValue - 0.01;
                launchZoneLED.setPosition(launchZoneValue);
            }
            if (gamepad1.dpadRightWasPressed()) {
                launchZoneValue = launchZoneValue + 0.01;
                launchZoneLED.setPosition(launchZoneValue);
            }

            TelemetryPacket colorValues = new TelemetryPacket();
            colorValues.put("LED1 Color Value: ", artifactLEDValue);
            colorValues.put("LED2 Color Value: ", launchZoneValue);
            FtcDashboard.getInstance().sendTelemetryPacket(colorValues);
            idle();

            telemetry.addData("LED1 Color Value: ", artifactLEDValue);
            telemetry.addData("LED2 Color Value: ", launchZoneValue);
            telemetry.update();
        }
    }
}
