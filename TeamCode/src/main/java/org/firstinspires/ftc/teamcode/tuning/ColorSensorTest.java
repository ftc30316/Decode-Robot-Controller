package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp

public class ColorSensorTest extends LinearOpMode {
    private NormalizedColorSensor artifactSlot1;
    private NormalizedColorSensor artifactSlot2;

    @Override
    public void runOpMode() {
        artifactSlot1 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor1");
        artifactSlot2 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor2");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) artifactSlot1).getLightDetected());
            NormalizedRGBA color1 = artifactSlot1.getNormalizedColors();

            telemetry.addData("Light Detected", ((OpticalDistanceSensor) artifactSlot2).getLightDetected());
            NormalizedRGBA color2 = artifactSlot2.getNormalizedColors();

            //Determining the amount of red, green, and blue
            telemetry.addData("Red", "%.3f", color1.red);
            telemetry.addData("Green", "%.3f", color1.green);
            telemetry.addData("Blue", "%.3f", color1.blue);

            telemetry.addData("Red", "%.3f", color2.red);
            telemetry.addData("Green", "%.3f", color2.green);
            telemetry.addData("Blue", "%.3f", color2.blue);

            //If blue is the greatest value, the color is purple. If blue is greater than red and green, the color is purple.
            //If green is the greatest value, the color is green. If green is greater than red and blue, the color is green.
            //If all colors are less than 0.01, no color is being sensed. If all colors are balanced and close to zero, no color is being sensed.

            colorDetection(color1);
            colorDetection(color2);

            telemetry.update();
        }
    }

    private void colorDetection(NormalizedRGBA colorSensor) {

        if (colorSensor.blue > colorSensor.green) {
            telemetry.addLine("The color is purple!");
        } else if (colorSensor.green > colorSensor.blue) {
            telemetry.addLine("The color is green!");
        } else {
            telemetry.addLine("No color is detected.");
        }

    }
}