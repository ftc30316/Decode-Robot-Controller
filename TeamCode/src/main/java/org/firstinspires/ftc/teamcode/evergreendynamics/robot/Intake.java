package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    private DcMotorEx intakeMotor;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Telemetry telemetry;
    private CRServo firstIntakeServo;
    private CRServo secondIntakeServo;
    private CRServo thirdIntakeServo;
    public DistanceSensor firstArtifactSensor;
    public DistanceSensor secondArtifactSensor;
    public DistanceSensor thirdArtifactSensor;
    private Servo artifactLED;
    private Servo launchZoneLED;

    public enum IntakeState {
        ON,

        OFF
    }

    IntakeState intakeState = IntakeState.ON;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

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
        launchZoneLED = hardwareMap.get(Servo.class, "launchZoneLED");

    }

    public void loop() {
        telemetry.addData("Intake state is: ", intakeState);
        telemetry.addData("Artifacts: ", getNumberOfArtifacts());
        turnOnLEDs();
        switch (intakeState) {
            case ON:
                intakeMotor.setPower(InputValues.INTAKE_POWER);
                firstIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
                secondIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
                thirdIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);

//                if (getNumberOfArtifacts() == 3) {
                if (gamepad1.squareWasPressed()) {
                    intakeState = IntakeState.OFF;
                }
                break;
            case OFF:
                intakeMotor.setPower(InputValues.INTAKE_POWER_SLOW);

                if (gamepad1.squareWasPressed()) {
                    intakeState = IntakeState.ON;
                }
                break;
//
//                if (getNumberOfArtifacts() < 3) {
//                    intakeState = IntakeState.ON;
//                }
        }
    }

    public void turnOnIntake() {
        intakeMotor.setPower(InputValues.INTAKE_POWER);
        firstIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
        secondIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
        thirdIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
    }

    public void turnOnLEDs () {
        // Controls the color of the LED that represents how many artifacts you have
        if (getNumberOfArtifacts() == 0) {
            artifactLED.setPosition(0.27); // Off
        }
        if (getNumberOfArtifacts() == 1) {
            artifactLED.setPosition(0.28); // Red
        }
        if (getNumberOfArtifacts() == 2) {
            artifactLED.setPosition(0.33); // Yellow
        }
        if (getNumberOfArtifacts() == 3) {
            artifactLED.setPosition(0.45); // Green
        }
    }

    public int getNumberOfArtifacts() {

        int numberOfArtifacts = 0;

        if (firstArtifactSensor.getDistance(DistanceUnit.INCH) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        if (secondArtifactSensor.getDistance(DistanceUnit.INCH) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        if (thirdArtifactSensor.getDistance(DistanceUnit.INCH) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        return 3;
    }
}