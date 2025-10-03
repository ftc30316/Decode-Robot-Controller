package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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


    //constructor:
    public Sorter(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1) {
        this.telemetry = telemetry;
        NormalizedColorSensor colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor1");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor2");
        NormalizedColorSensor colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "colorsensor3");
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo servo3 = hardwareMap.get(Servo.class, "servo3");

        leftSlot = new Slot(telemetry, colorSensor1, servo1, gamepad1);
        middleSlot = new Slot(telemetry, colorSensor2, servo2, gamepad1);
        rightSlot = new Slot(telemetry, colorSensor3, servo3, gamepad1);
    }

    public void detect(){
        leftSlot.sort();
        middleSlot.sort();
        rightSlot.sort();
    }
}
