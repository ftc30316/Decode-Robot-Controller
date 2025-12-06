package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled

public class ArtifactSorterTest extends LinearOpMode {
    private NormalizedColorSensor artifactSlot1;
    private NormalizedColorSensor artifactSlot2;
    private NormalizedColorSensor artifactSlot3;
    private String nextColorArtifact = "purple";

    enum State {
        DETECTING,
        HOLDING,
        FLICKING,
        RESETTING
    }

    State sorterState = State.DETECTING;
    ElapsedTime flickTimer = new ElapsedTime();  // Timer to track time in each state

    private final double FLICK_DISTANCE = 0.5;
    private final double TRAVEL_TIME = 3.0;

    @Override
    public void runOpMode() {
        artifactSlot1 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor1");
        artifactSlot2 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor2");
        artifactSlot3 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor3");
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo servo3 = hardwareMap.get(Servo.class, "servo3");

        servo1.setPosition(0);
        servo2.setPosition(0);
        servo3.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) artifactSlot1).getLightDetected());
            NormalizedRGBA color1 = artifactSlot1.getNormalizedColors();

            telemetry.addData("Light Detected", ((OpticalDistanceSensor) artifactSlot2).getLightDetected());
            NormalizedRGBA color2 = artifactSlot2.getNormalizedColors();

            telemetry.addData("Light Detected", ((OpticalDistanceSensor) artifactSlot3).getLightDetected());
            NormalizedRGBA color3 = artifactSlot3.getNormalizedColors();

            //Determining the amount of red, green, and blue
            telemetry.addData("Red1", color1.red);
            telemetry.addData("Green1", color1.green);
            telemetry.addData("Blue1", color1.blue);

//            telemetry.addData("Red2", color2.red);
//            telemetry.addData("Green2", color2.green);
//            telemetry.addData("Blue2", color2.blue);
//
//            telemetry.addData("Red3", color3.red);
//            telemetry.addData("Green3", color3.green);
//            telemetry.addData("Blue3", color3.blue);

            telemetry.update();

//            String sensor2color = colorDetection(color2);
//            String sensor3color = colorDetection(color3);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Red1", color1.red);
            packet.put("Green1", color1.green);
            packet.put("Blue1", color1.blue);

//            packet.put("Red2", color2.red);
//            packet.put("Green2", color2.green);
//            packet.put("Blue2", color2.blue);
//
//            packet.put("Red3", color3.red);
//            packet.put("Green3", color3.green);
//            packet.put("Blue3", color3.blue);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

//            if (sensor1color != null && sensor1color.equals(nextColorArtifact)) {
//                servo1.setPosition(1);
//                nextColorArtifact = "green";
//
//            } else if (sensor2color != null && sensor2color.equals(nextColorArtifact)) {
//                servo2.setPosition(1);
//                nextColorArtifact = "green";
//            } else if (sensor3color != null && sensor3color.equals(nextColorArtifact)) {
//                servo3.setPosition(1);
//                nextColorArtifact = "green";
//            }

            telemetry.addData("Current State ", sorterState);
            switch (sorterState) {
                case DETECTING:
                    String sensor1color = colorDetection(color1);
                    telemetry.addLine("The detecting color is "+ sensor1color);
                    if (sensor1color != null) {
                        sorterState = State.HOLDING;
                    }
                    break;
                case HOLDING:
                    String sensor1colorHolding = colorDetection(color1);
                    telemetry.addLine("The holding color is "+ sensor1colorHolding);
                    if (sensor1colorHolding != null && sensor1colorHolding.equals(nextColorArtifact) && gamepad1.right_trigger == 1.0) {
                        sorterState = State.FLICKING;
                    }
                    break;
                case FLICKING:
                    flickTimer.reset();
                    servo1.setPosition(FLICK_DISTANCE);
                    sorterState = State.RESETTING;
                    break;
                case RESETTING:
                    if (flickTimer.seconds() > TRAVEL_TIME) {
                        servo1.setPosition(0.0);
                        sorterState = State.DETECTING;
                    }
                    break;
            }

        }
    }

    private String colorDetection(NormalizedRGBA colorSensor) {

        //If blue is the greatest value, the color is purple. If blue is greater than red and green, the color is purple.
        //If green is the greatest value, the color is green. If green is greater than red and blue, the color is green.
        //If all colors are less than 0.01, no color is being sensed. If all colors are balanced and close to zero, no color is being sensed.

        if (colorSensor.red < 0.01 && colorSensor.blue < 0.01 && colorSensor.green < 0.01){
            telemetry.addLine("No color is detected.");
            return null;
        } else if (colorSensor.green > colorSensor.blue) {
            telemetry.addLine("The color is green!");
            return "green";
        } else if (colorSensor.blue > colorSensor.green) {
            telemetry.addLine("The color is purple!");
            return "purple";
        } else {
            telemetry.addLine("Cannot detect color.");
            return null;
        }

    }
}