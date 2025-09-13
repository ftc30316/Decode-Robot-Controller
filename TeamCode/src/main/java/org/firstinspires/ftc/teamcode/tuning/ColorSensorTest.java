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
            telemetry.addData("Red1", color1.red);
            telemetry.addData("Green1", color1.green);
            telemetry.addData("Blue1", color1.blue);

            telemetry.addData("Red2", color2.red);
            telemetry.addData("Green2", color2.green);
            telemetry.addData("Blue2", color2.blue);


            colorDetection(color1);
            colorDetection(color2);

            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Red1", color1.red);
            packet.put("Green1", color1.green);
            packet.put("Blue1", color1.blue);

            packet.put("Red2", color2.red);
            packet.put("Green2", color2.green);
            packet.put("Blue2", color2.blue);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private void colorDetection(NormalizedRGBA colorSensor) {

        //If blue is the greatest value, the color is purple. If blue is greater than red and green, the color is purple.
        //If green is the greatest value, the color is green. If green is greater than red and blue, the color is green.
        //If all colors are less than 0.01, no color is being sensed. If all colors are balanced and close to zero, no color is being sensed.

        if (colorSensor.red < 0.01 && colorSensor.blue < 0.01 && colorSensor.green < 0.01){
            telemetry.addLine("No color is detected.");
        } else if (colorSensor.green > colorSensor.blue) {
            telemetry.addLine("The color is green!");
        } else if (colorSensor.blue > colorSensor.green) {
            telemetry.addLine("The color is purple!");
        } else {
            telemetry.addLine("Cannot detect color.");
        }

    }
}