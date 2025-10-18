package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sorter {
    Slot leftSlot;
    Slot middleSlot;
    Slot rightSlot;

    private Telemetry telemetry;

    public volatile Gamepad gamepad1 = null;
    private DistanceSensor distanceSensor;


    //constructor:
    public Sorter(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        NormalizedColorSensor colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor1");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor2");
        NormalizedColorSensor colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor3");
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo servo3 = hardwareMap.get(Servo.class, "servo3");
        DistanceSensor distanceSensor1 = hardwareMap.get(DistanceSensor.class, "colorsensor1");
        DistanceSensor distanceSensor2 = hardwareMap.get(DistanceSensor.class, "colorsensor2");
        DistanceSensor distanceSensor3 = hardwareMap.get(DistanceSensor.class, "colorsensor3");

        leftSlot = new Slot(telemetry, colorSensor1, servo1, gamepad1, distanceSensor1, true);
        middleSlot = new Slot(telemetry, colorSensor2, servo2, gamepad1, distanceSensor2, true);
        rightSlot = new Slot(telemetry, colorSensor3, servo3, gamepad1, distanceSensor3, false);
    }


    public void detect() {
        if (gamepad1.left_bumper) {
            flickArtifactGreen();
        }

        if (gamepad1.right_bumper) {
            flickArtifactPurple();
        }

        leftSlot.sort();
        middleSlot.sort();
        rightSlot.sort();
    }

    public void flickArtifactGreen () {
        String leftColor = leftSlot.getColorDetected();
        String middleColor = middleSlot.getColorDetected();
        String rightColor = rightSlot.getColorDetected();

        if (leftColor.equals("green")) {
            leftSlot.switchToFlicking();
        } else if (middleColor.equals("green")) {
            middleSlot.switchToFlicking();
        } else if (rightColor.equals("green")) {
            rightSlot.switchToFlicking();
        }
    }

    public void flickArtifactPurple () {
        String leftColor = leftSlot.getColorDetected();
        String middleColor = middleSlot.getColorDetected();
        String rightColor = rightSlot.getColorDetected();

        if (leftColor.equals("purple")) {
            leftSlot.switchToFlicking();
        } else if (middleColor.equals("purple")) {
            middleSlot.switchToFlicking();
        } else if (rightColor.equals("purple")) {
            rightSlot.switchToFlicking();
        }
    }
}
