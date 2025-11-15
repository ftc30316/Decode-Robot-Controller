package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    public Thread sorterBackgroundThread;

    private Telemetry telemetry;

    public volatile Gamepad gamepad1 = null;
    public volatile Gamepad gamepad2 = null;

    // Sets up the three different slots, each with their own servo, orientation, and color/distance sensor
    public Sorter(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        NormalizedColorSensor colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "middleColorSensor");
        NormalizedColorSensor colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");
        Servo servo1 = hardwareMap.get(Servo.class, "leftServo");
        Servo servo2 = hardwareMap.get(Servo.class, "middleServo");
        Servo servo3 = hardwareMap.get(Servo.class, "rightServo");
        Servo leftFlipServo = hardwareMap.get(Servo.class, "leftFlipServo");
        Servo rightFlipServo = hardwareMap.get(Servo.class, "rightFlipServo");

        leftSlot = new Slot(telemetry, colorSensor1, servo1, leftFlipServo, rightFlipServo, gamepad1, gamepad2, Slot.Orientation.LEFT);
        middleSlot = new Slot(telemetry, colorSensor2, servo2, leftFlipServo, rightFlipServo, gamepad1, gamepad2, Slot.Orientation.MIDDLE);
        rightSlot = new Slot(telemetry, colorSensor3, servo3, leftFlipServo, rightFlipServo, gamepad1, gamepad2, Slot.Orientation.RIGHT);

        // Creates a background thread so that the flippers can flip and the flickers can flick at the same time
        // this.sorterBackgroundThread = new Thread(this:://TODO fill this in);
    }

    // Based on gamepad trigger, asks slots for a certain colored artifact
    public void detect () {
// This code goes based off of the color sensor data. We found this unreliable, so we switched to gamepad two determining whether we should flick the left, middle, or right slot.
//        if (gamepad1.left_bumper) {
//            flickArtifactGreen();
//        }
//        if (gamepad1.right_bumper) {
//            flickArtifactPurple();
//        }
        if (gamepad2.square) {
            leftSlot.switchToFlicking();
        }
        else if (gamepad2.cross) {
            middleSlot.switchToFlicking();
        }
        else if (gamepad2.circle) {
            rightSlot.switchToFlicking();
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

    public void flickAll () { // last resort flicking
        leftSlot.switchToFlicking();
        middleSlot.switchToFlicking();
        rightSlot.switchToFlicking();
    }
}
