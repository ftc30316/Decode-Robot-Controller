package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intake1;
    private Gamepad gamepad1;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1){
        intake1 = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        this.gamepad1 = gamepad1;


    }

    public void startSpin(){
        intake1.setVelocity(InputValues.INTAKE_SPEED);

    }
}
