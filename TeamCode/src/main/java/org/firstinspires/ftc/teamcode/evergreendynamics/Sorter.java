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
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor1");
        Servo servo = hardwareMap.get(Servo.class, "servo1");

        leftSlot = new Slot(telemetry, colorSensor, servo, gamepad1);
    }

    public void detect(){
        leftSlot.sort();
    }
}
