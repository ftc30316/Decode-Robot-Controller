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

    // Sets up the three different slots, each with their own servo, orientation, and color/distance sensor
    public Sorter(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        NormalizedColorSensor colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "middleColorSensor");
        NormalizedColorSensor colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");
        Servo servo1 = hardwareMap.get(Servo.class, "leftServo");
        Servo servo2 = hardwareMap.get(Servo.class, "middleServo");
        Servo servo3 = hardwareMap.get(Servo.class, "rightServo");
        DistanceSensor distanceSensor1 = hardwareMap.get(DistanceSensor.class, "leftColorSensor");
        DistanceSensor distanceSensor2 = hardwareMap.get(DistanceSensor.class, "middleColorSensor");
        DistanceSensor distanceSensor3 = hardwareMap.get(DistanceSensor.class, "rightColorSensor");

        leftSlot = new Slot(telemetry, colorSensor1, servo1, gamepad1, distanceSensor1, Slot.Orientation.LEFT);
        middleSlot = new Slot(telemetry, colorSensor2, servo2, gamepad1, distanceSensor2, Slot.Orientation.MIDDLE);
        rightSlot = new Slot(telemetry, colorSensor3, servo3, gamepad1, distanceSensor3, Slot.Orientation.RIGHT);
    }

    // Based on gamepad trigger, asks slots for a certain colored artifact
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

    // Determines which slots have a green artifact, and moves whichever servo has the first green artifact
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

    // Determines which slots have a purple artifact, and moves whichever servo has the first purple artifact
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
