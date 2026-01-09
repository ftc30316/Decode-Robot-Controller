package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import android.renderscript.ScriptGroup;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    public Keybinds keybinds;
    private DcMotorEx intakeMotor;
    private Telemetry telemetry;
    private CRServo firstIntakeServo;
    private CRServo secondIntakeServo;
    private CRServo thirdIntakeServo;
    public DistanceSensor firstArtifactSensor;
    public DistanceSensor secondArtifactSensor;
    public DistanceSensor thirdArtifactSensor;
    private Servo artifactLED;

    public enum IntakeState {
        ON,

        OFF
    }

    IntakeState intakeState = IntakeState.ON;

    public Intake(HardwareMap hardwareMap, Keybinds keybinds, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.telemetry = telemetry;
        this.keybinds = keybinds;

        firstIntakeServo = hardwareMap.get(CRServo.class, "frontTunnelServo");
        secondIntakeServo = hardwareMap.get(CRServo.class, "middleTunnelServo");
        thirdIntakeServo = hardwareMap.get(CRServo.class, "backTunnelServo");

        firstIntakeServo.setDirection(CRServo.Direction.REVERSE);
        secondIntakeServo.setDirection(CRServo.Direction.REVERSE);
        thirdIntakeServo.setDirection(CRServo.Direction.REVERSE);

        firstArtifactSensor = hardwareMap.get(DistanceSensor.class, "frontArtifactSensor");
        secondArtifactSensor = hardwareMap.get(DistanceSensor.class, "middleArtifactSensor");
        thirdArtifactSensor = hardwareMap.get(DistanceSensor.class, "backArtifactSensor");

        artifactLED = hardwareMap.get(Servo.class, "artifactLED");

    }

    public void loop() {
//        telemetry.addData("Intake state is: ", intakeState);
        telemetry.addData("Artifacts: ", getNumberOfArtifacts());
        turnOnLEDs();
        switch (intakeState) {
            case ON:
//                intakeMotor.setPower(InputValues.INTAKE_POWER);
                intakeMotor.setVelocity(InputValues.INTAKE_VELOCITY);
                firstIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
                secondIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
                thirdIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);

//                if (getNumberOfArtifacts() == 3) {
//                if (keybinds.changeIntakeState()) {
//                    intakeState = IntakeState.OFF;
//                }
                break;
            case OFF:
                intakeMotor.setPower(0.0);
//                if (keybinds.changeIntakeState()) {
//                    intakeState = IntakeState.ON;
//                }
//
//                if (getNumberOfArtifacts() < 3) {
//                    intakeState = IntakeState.ON;
//                }
                break;
        }
    }

    public void disableIntake() {
        intakeState = IntakeState.OFF;
    }

    public void turnOnIntake() {
        intakeMotor.setPower(InputValues.INTAKE_POWER);
        firstIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
        secondIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
        thirdIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
    }

    public void turnOnLEDs() {
        // Controls the color of the LED that represents how many artifacts you have
        if (getNumberOfArtifacts() == 0) {
            artifactLED.setPosition(InputValues.OFF);
        }
        if (getNumberOfArtifacts() == 1) {
            artifactLED.setPosition(InputValues.RED);
        }
        if (getNumberOfArtifacts() == 2) {
            artifactLED.setPosition(InputValues.YELLOW);
        }
        if (getNumberOfArtifacts() == 3) {
            artifactLED.setPosition(InputValues.GREEN);
        }
    }

    public int getNumberOfArtifacts() {

        int numberOfArtifacts = 0;

        if (Helper.getAverageDistance(firstArtifactSensor) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        if (Helper.getAverageDistance(secondArtifactSensor) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        if (Helper.getAverageDistance(thirdArtifactSensor) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        return numberOfArtifacts;
//        return 3;
    }


}