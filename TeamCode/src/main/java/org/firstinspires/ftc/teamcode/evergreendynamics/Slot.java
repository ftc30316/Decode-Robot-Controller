package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tuning.ArtifactSorterTest;


public class Slot {
    public enum State {
        DETECTING,
        HOLDING,
        FLICKING,
        RESETTING
    }
    public State sorterState = State.DETECTING;
    public ElapsedTime flickTimer = new ElapsedTime();  // Timer to track time in each state

    private final double FLICK_DISTANCE = 0.05;
    private final double TRAVEL_TIME = 3.0;

    private Telemetry telemetry;

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;


    private Servo servo;

    private String nextColorArtifact = "green";

    public volatile Gamepad gamepad1 = null;

    public Slot(Telemetry t, NormalizedColorSensor colorSensor, Servo servo, Gamepad gamepad1, DistanceSensor distanceSensor) {
        this.telemetry = t;
        this.colorSensor = colorSensor;
        this.servo = servo;
        this.gamepad1 = gamepad1;
        this.distanceSensor = distanceSensor;


    }
    public void sort() {
        telemetry.addData("Current State ", sorterState);
        telemetry.addData("slot distance", distanceSensor.getDistance(DistanceUnit.INCH));
        switch (sorterState) {
            case DETECTING:
                String sensor1color = colorDetection(colorSensor);
                telemetry.addLine("The detecting color is "+ sensor1color);
                if (sensor1color != null) {
                    sorterState = State.HOLDING;
                }
                break;
            case HOLDING:
                String sensor1colorHolding = colorDetection(colorSensor);
                telemetry.addLine("The holding color is "+ sensor1colorHolding);
                if (sensor1colorHolding != null && sensor1colorHolding.equals(nextColorArtifact) && gamepad1.right_trigger == 1.0) {
                    sorterState = State.FLICKING;
                }
//                if (distanceSensor.getDistance(DistanceUnit.INCH) < 5.5 && gamepad1.right_trigger == 1.0) {
//                    sorterState = State.FLICKING;
//                }
                break;
            case FLICKING:
                flickTimer.reset();
                servo.setPosition(FLICK_DISTANCE);
                sorterState = State.RESETTING;
                break;
            case RESETTING:
                if (flickTimer.seconds() > TRAVEL_TIME) {
                    servo.setPosition(0.0);
                    sorterState = State.DETECTING;
                }
                break;

        }
    }

    private String colorDetection(NormalizedColorSensor colorSensor) {

        //If blue is the greatest value, the color is purple. If blue is greater than red and green, the color is purple.
        //If green is the greatest value, the color is green. If green is greater than red and blue, the color is green.
        //If all colors are less than 0.01, no color is being sensed. If all colors are balanced and close to zero, no color is being sensed.
        NormalizedRGBA color1 = colorSensor.getNormalizedColors();
        if (color1.red < 0.01 && color1.blue < 0.01 && color1.green < 0.01){
            telemetry.addLine("No color is detected.");
            return null;
        } else if (color1.green > color1.blue) {
            telemetry.addLine("The color is green!");
            return "green";
        } else if (color1.blue > color1.green) {
            telemetry.addLine("The color is purple!");
            return "purple";
        } else {
            telemetry.addLine("Cannot detect color.");
            return null;
        }

    }
}
