package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public enum IntakeState {
        ON,

        OFF
    }

    IntakeState intakeState = IntakeState.ON;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

    }

    public void loop() {
        telemetry.addData("Intake state is: ", intakeState);
        telemetry.addData("Artifacts: ", getNumberOfArtifacts());
        switch (intakeState) {
            case ON:
                intakeMotor.setVelocity(InputValues.INTAKE_SPEED);
                firstIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
                secondIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
                thirdIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);

                if (getNumberOfArtifacts() == 3) {
                    intakeState = IntakeState.OFF;
                }
                break;
            case OFF:
                intakeMotor.setVelocity(0);

                if (getNumberOfArtifacts() < 3) {
                    intakeState = IntakeState.ON;
                }
        }
    }

    public void turnOnIntake () {
        intakeMotor.setVelocity(InputValues.INTAKE_SPEED);
        firstIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
        secondIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
        thirdIntakeServo.setPower(InputValues.INTAKE_SERVO_POWER);
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
        return numberOfArtifacts;
    }
}