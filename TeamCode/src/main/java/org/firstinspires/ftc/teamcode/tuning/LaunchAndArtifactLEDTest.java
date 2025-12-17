


package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class LaunchAndArtifactLEDTest extends LinearOpMode {
    private Servo LED1;
    private Servo LED2;
    int NumberOfArtifacts = 0;
    boolean InLaunchZone = false;

    @Override
    public void runOpMode() {

        LED1 = hardwareMap.get(Servo.class, "LED1");
        LED2 = hardwareMap.get(Servo.class, "LED2");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("To turn the launch zone light on, press left on the dpad. To turn off, press right on the dpad. Pink = In launch zone, Off = Not in launch zone");
            telemetry.addLine("To add an artifact, press up on the dpad. To subtract an artifact, press down on the dpad.");
            telemetry.addLine("Off = Zero artifacts, Red = One artifact, Yellow = Two artifacts, Green = Three artifacts");

            // Controls how many artifacts you have
            if (gamepad1.dpadUpWasPressed()) {
                NumberOfArtifacts = NumberOfArtifacts + 1;
            }
            if (gamepad1.dpadDownWasPressed()) {
                NumberOfArtifacts = NumberOfArtifacts - 1;
            }

            // Controls if you are in the launch zone or not
            if (gamepad1.dpadLeftWasPressed()) {
                InLaunchZone = true;
            }
            if (gamepad1.dpadRightWasPressed()) {
                InLaunchZone = false;
            }

            // Controls the color of the LED that represents how many artifacts you have
            if (NumberOfArtifacts == 0) {
                LED1.setPosition(0.27); // Off
            }
            if (NumberOfArtifacts == 1) {
                LED1.setPosition(0.28); // Red
            }
            if (NumberOfArtifacts == 2) {
                LED1.setPosition(0.33); // Yellow
            }
            if (NumberOfArtifacts >= 3) {
                LED1.setPosition(0.45); // Green
            }

            // Controls the color of the LED that represents if you are in the launch zone or not
            if (InLaunchZone) {
                LED2.setPosition(0.72); // Pink
            }
            if (!InLaunchZone) {
                LED2.setPosition(0.27); // Off
            }

            telemetry.addData("Number of artifacts: ", NumberOfArtifacts);
            telemetry.addData("In launch zone: ", InLaunchZone);
            telemetry.update();
        }
    }
}
