package org.firstinspires.ftc.teamcode.evergreendynamics;

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
        Servo leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");
        Servo rightGateServo = hardwareMap.get(Servo.class, "rightGateServo");

        leftSlot = new Slot(telemetry, colorSensor1, servo1, leftGateServo, rightGateServo, gamepad1, gamepad2, Slot.Orientation.LEFT);
        middleSlot = new Slot(telemetry, colorSensor2, servo2, leftGateServo, rightGateServo, gamepad1, gamepad2, Slot.Orientation.MIDDLE);
        rightSlot = new Slot(telemetry, colorSensor3, servo3, leftGateServo, rightGateServo, gamepad1, gamepad2, Slot.Orientation.RIGHT);

    }

    // Based on gamepad trigger, asks slots for a certain colored artifact
    public void detect () {
// This code goes based off of the color sensor data. We found this unreliable, so we switched to gamepad two determining whether we should flick the left, middle, or right slot.
        if (gamepad1.left_bumper) {
            switchGreenSlotState();
        }
        if (gamepad1.right_bumper) {
            switchPurpleSlotState();
        }
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
    public void switchGreenSlotState() {
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
    public void switchPurpleSlotState() {
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

    public void flickAll() { // last resort flicking
        leftSlot.switchToFlicking();
        middleSlot.switchToFlicking();
        rightSlot.switchToFlicking();
    }

    public void flickGreenSlot() {
        // triggers flick
        switchGreenSlotState();
        flickSlot();
    }

    public void flickPurpleSlot() {
        // triggers flick
        switchPurpleSlotState();
        flickSlot();
    }

    public void flickSlot() {
        detect();
        try {
            Thread.sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        // triggers reset
        detect();
        try {
            Thread.sleep((long) (InputValues.SETTLE_TIME * 1000));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void flickForMotif(int motifTagId, int shotNumber) {
        // changes state to flicking
        if (shotNumber == 1) {
            if (motifTagId == 21) {
                flickGreenSlot();
            } else if ((motifTagId == 22) || (motifTagId == 23)) {
                flickPurpleSlot();
            }
        }
        else if (shotNumber == 2) {
            if (motifTagId == 22) {
                flickGreenSlot();
            } else if ((motifTagId == 21) || (motifTagId == 23)) {
                flickPurpleSlot();
            }
        }
        else if (shotNumber == 3) {
            if (motifTagId == 23) {
                flickGreenSlot();
            } else if ((motifTagId == 21) || (motifTagId == 22)) {
                flickPurpleSlot();
            }
        }
    }

    public void backupFlickAll() {
        // Safety net, flicks all just in case
        flickAll();
        detect(); // triggers flick
        try {
            Thread.sleep((long) (InputValues.FLICK_TRAVEL_TIME * 1000));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        detect(); // triggers reset
        try {
            Thread.sleep((long) (InputValues.SETTLE_TIME * 1000));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }
}
