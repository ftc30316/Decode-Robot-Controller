
// To make LED 1 go through the color cycle, press up on the dpad. Press down to go in reverse.
// To make LED 2 go through the color cycle, press right on the dpad. Press left to go in reverse.

// 0.27 and below is off
// 0.73 and above is white

package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class LEDValueTest extends LinearOpMode {
    private Servo LED1;
    private Servo LED2;
    double LED1Value = 0.25;
    double LED2Value = 0.25;

    @Override
    public void runOpMode() {

        LED1 = hardwareMap.get(Servo.class, "LED1");
        LED2 = hardwareMap.get(Servo.class, "LED2");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpadUpWasPressed()) {
                LED1Value = LED1Value + 0.01;
                LED1.setPosition(LED1Value);
            }
            if (gamepad1.dpadDownWasPressed()) {
                LED1Value = LED1Value - 0.01;
                LED1.setPosition(LED1Value);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                LED2Value = LED2Value - 0.01;
                LED2.setPosition(LED2Value);
            }
            if (gamepad1.dpadRightWasPressed()) {
                LED2Value = LED2Value + 0.01;
                LED2.setPosition(LED2Value);
            }

            TelemetryPacket colorValues = new TelemetryPacket();
            colorValues.put("LED1 Color Value: ", LED1Value);
            colorValues.put("LED2 Color Value: ", LED2Value);
            FtcDashboard.getInstance().sendTelemetryPacket(colorValues);
            idle();

            telemetry.addData("LED1 Color Value: ", LED1Value);
            telemetry.addData("LED2 Color Value: ", LED2Value);
            telemetry.update();
        }
    }
}
