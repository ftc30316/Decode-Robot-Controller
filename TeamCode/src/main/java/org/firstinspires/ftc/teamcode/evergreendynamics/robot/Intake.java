package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotorEx intakeMotor;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Telemetry telemetry;
    private CRServo beltServo;
    public DistanceSensor firstArtifactSensor;
    public DistanceSensor secondArtifactSensor;
    public DistanceSensor thirdArtifactSensor;

    public enum IntakeState {
        ON,

        REVERSE,

        OFF
    }

    IntakeState intakeState = IntakeState.ON;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        beltServo = hardwareMap.get(CRServo.class, "beltServo");
        firstArtifactSensor = hardwareMap.get(DistanceSensor.class, "firstArtifactSensor");
        secondArtifactSensor = hardwareMap.get(DistanceSensor.class, "secondArtifactSensor");
        thirdArtifactSensor = hardwareMap.get(DistanceSensor.class, "thirdArtifactSensor");

    }

    public void startSpin() {
        intakeMotor.setVelocity(InputValues.INTAKE_SPEED);
        beltServo.setPower(InputValues.BELT_SERVO_POWER);
    }

    public void triggerIntake() {
        telemetry.addData("Intake state is: ", intakeState);
        switch (intakeState) {
            case ON:
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeMotor.setVelocity(InputValues.INTAKE_SPEED);
                beltServo.setDirection(CRServo.Direction.REVERSE);
                beltServo.setPower(InputValues.BELT_SERVO_POWER);

                if (gamepad2.dpad_down) {
                    intakeState = IntakeState.REVERSE;
                }
                break;
            case REVERSE:
                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeMotor.setVelocity(InputValues.SLOW_INTAKE_SPEED);
                beltServo.setDirection(CRServo.Direction.FORWARD);
                beltServo.setPower(InputValues.SLOW_BELT_SERVO_POWER);
                if (gamepad2.dpad_up) {
                    intakeState = IntakeState.ON;
                }
                break;
            case OFF:
                intakeMotor.setVelocity(0);
        }
    }
}