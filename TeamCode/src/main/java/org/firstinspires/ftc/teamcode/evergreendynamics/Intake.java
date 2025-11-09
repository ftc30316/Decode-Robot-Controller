package org.firstinspires.ftc.teamcode.evergreendynamics;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotorEx intake1;
    private Gamepad gamepad1;
    private Telemetry telemetry;

    private CRServo rightIntakeServo;

    private CRServo leftIntakeServo;

    public enum IntakeState {
        ON,
        OFF,
        REVERSE
    }

    IntakeState intakeState = IntakeState.ON;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        intake1 = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");


    }

    public void startSpin() {
        intake1.setVelocity(InputValues.INTAKE_SPEED);
    }
    public void triggerIntake() {
        telemetry.addData("Intake state is: ", intakeState);
        switch (intakeState) {
            case OFF:
                intake1.setVelocity(0);
                if (gamepad1.square) {
                    intakeState = IntakeState.ON;
                }
                break;
            case ON:
                intake1.setDirection(DcMotorSimple.Direction.REVERSE);
                intake1.setVelocity(InputValues.INTAKE_SPEED);
                rightIntakeServo.setDirection(CRServo.Direction.REVERSE);
                rightIntakeServo.setPower(1.0);
                leftIntakeServo.setPower(1.0);

                if (gamepad1.square) {
                    intakeState = IntakeState.REVERSE;
                }

                break;
            case REVERSE:
                intake1.setDirection(DcMotorSimple.Direction.FORWARD);
                intake1.setVelocity(InputValues.SLOW_INTAKE_SPEED);
                if (gamepad1.square) {
                    intakeState = IntakeState.OFF;
                }
                break;
        }
    }
}