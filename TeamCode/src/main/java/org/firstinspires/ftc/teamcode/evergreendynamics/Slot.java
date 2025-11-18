package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Slot {
    public enum State {
        DETECTING,
        HOLDING,
        FLICKING,
        RESETTING
    }

    public enum Orientation {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public State sorterState = State.DETECTING;
    public Orientation slotOrientation = Orientation.LEFT;
    public ElapsedTime flickTimer = new ElapsedTime();  // Timer to track time in each state
    private Telemetry telemetry;
    private NormalizedColorSensor colorSensor;
    private Servo servo;
    private Servo leftGateServo;
    private Servo rightGateServo;

    public volatile Gamepad gamepad1 = null;
    public volatile Gamepad gamepad2 = null;
    public String slotColor = "No Artifact";

    public Slot(Telemetry t, NormalizedColorSensor colorSensor, Servo servo, Servo leftGateServo, Servo rightGateServo, Gamepad gamepad1, Gamepad gamepad2, Orientation slotOrientation) {
        this.telemetry = t;
        this.colorSensor = colorSensor;
        this.servo = servo;
        this.leftGateServo = leftGateServo;
        this.rightGateServo = rightGateServo;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.slotOrientation = slotOrientation;

        // Based on orientation of servo, moves to zero
        if (slotOrientation == Orientation.LEFT) {
            servo.setPosition(InputValues.RESET_POS_LEFT);
            leftGateServo.setPosition(InputValues.CLOSED_GATE_POS_LEFT);
        }
        else if (slotOrientation == Orientation.MIDDLE) {
            servo.setPosition((InputValues.RESET_POS_MIDDLE));
        }
        else if (slotOrientation == Orientation.RIGHT) {
            servo.setPosition(InputValues.RESET_POS_RIGHT);
            rightGateServo.setPosition(InputValues.CLOSED_GATE_POS_RIGHT);
        }
    }

    // Creates a state machine that determines if there is an artifact, what color it is, and whether or not the driver has pressed the trigger to shoot
    public void sort() {
        telemetry.addData(slotOrientation.name() + " current state: ", sorterState);
        //telemetry.addData(slotOrientation.name() + " distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
        switch (sorterState) {
            case DETECTING:
                slotColor = colorDetection(colorSensor);
                telemetry.addLine(slotOrientation.name() + " detecting color is: "+ slotColor);
                if (slotColor.equals("green") || slotColor.equals("purple")) {
                    sorterState = State.HOLDING;
                }
                break;
            case HOLDING:
                telemetry.addLine(slotOrientation.name() + " the holding color is: "+ slotColor);
                if (gamepad1.right_trigger == 1.0) {
                    sorterState = State.FLICKING;
                }
                break;
            case FLICKING:
                flickTimer.reset();

                if (slotOrientation == Orientation.LEFT) {
                    servo.setPosition(InputValues.FLICK_POS_LEFT);
                    leftGateServo.setPosition(InputValues.OPEN_GATE_POS_LEFT);
                }
                else if (slotOrientation == Orientation.MIDDLE) {
                    servo.setPosition(InputValues.FLICK_POS_MIDDLE);
                    leftGateServo.setPosition(InputValues.OPEN_GATE_POS_LEFT);
                    rightGateServo.setPosition(InputValues.OPEN_GATE_POS_RIGHT);
                }
                else if (slotOrientation == Orientation.RIGHT) {
                    servo.setPosition(InputValues.FLICK_POS_RIGHT);
                    rightGateServo.setPosition(InputValues.OPEN_GATE_POS_RIGHT);
                }
                slotColor = "No Artifact";
                sorterState = State.RESETTING;
                break;
            case RESETTING:
                // Once the time elapsed is greater than the required time for the servo to move, it will reset the servo
                if (flickTimer.seconds() > InputValues.FLICK_TRAVEL_TIME) {
                    if (slotOrientation == Orientation.LEFT) {
                        servo.setPosition(InputValues.RESET_POS_LEFT);
                        leftGateServo.setPosition(InputValues.CLOSED_GATE_POS_LEFT);
                    }
                    else if (slotOrientation == Orientation.MIDDLE) {
                        servo.setPosition((InputValues.RESET_POS_MIDDLE));
                        leftGateServo.setPosition(InputValues.CLOSED_GATE_POS_LEFT);
                        rightGateServo.setPosition(InputValues.CLOSED_GATE_POS_RIGHT);
                    }
                    else if (slotOrientation == Orientation.RIGHT) {
                        servo.setPosition(InputValues.RESET_POS_RIGHT);
                        rightGateServo.setPosition(InputValues.CLOSED_GATE_POS_RIGHT);
                    }
                    sorterState = State.DETECTING;
                }
                break;

        }
    }

    // Asks the color sensor what color artifact is in the slot, if there is an artifact
    public String getColorDetected() {
            return slotColor;
    }

    // Switches slot state to flicking, where the servos will move to different positions based on their orientation
    public void switchToFlicking() {
        sorterState = State.FLICKING;
    }

    // Uses the data from the color sensors to determine the artifact color
    private String colorDetection(NormalizedColorSensor colorSensor) {

        //If blue is the greatest value, the color is purple. If blue is greater than red and green, the color is purple.
        //If green is the greatest value, the color is green. If green is greater than red and blue, the color is green.
        //If all colors are less than 0.01, no color is being sensed. If all colors are balanced and close to zero, no color is being sensed.
        NormalizedRGBA color1 = colorSensor.getNormalizedColors();
        if (color1.red < 0.01 && color1.blue < 0.01 && color1.green < 0.01){
            telemetry.addLine("No color is detected.");
            return "No Artifact";
        } else if (color1.green > color1.blue) {
            telemetry.addLine("The color is green!");
            return "green";
        } else if (color1.blue > color1.green) {
            telemetry.addLine("The color is purple!");
            return "purple";
        } else {
            telemetry.addLine("Cannot detect color.");
            return "Unknown color";
        }

    }
}
