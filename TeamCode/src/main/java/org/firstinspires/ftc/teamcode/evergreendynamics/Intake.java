package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intake1;
    private Gamepad gamepad1;

    public enum IntakeState {
        ON,
        OFF,
        REVERSE
    }

    IntakeState intakeState = IntakeState.OFF;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1) {
        intake1 = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.gamepad1 = gamepad1;


    }

    public void startSpin() {
        intake1.setVelocity(InputValues.INTAKE_SPEED);
    }
    public void triggerIntake() {
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
                if (gamepad1.square) {
                    intakeState = IntakeState.REVERSE;
                }
                break;
            case REVERSE:
                intake1.setDirection(DcMotorSimple.Direction.FORWARD);
                intake1.setVelocity(InputValues.INTAKE_SPEED);
                if (gamepad1.square) {
                    intakeState = IntakeState.OFF;
                }
                break;
        }
    }
}