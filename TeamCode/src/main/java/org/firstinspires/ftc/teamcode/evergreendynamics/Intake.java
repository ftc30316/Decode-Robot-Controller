package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake1;

    public Intake(HardwareMap hardwareMap){
        intake1 = hardwareMap.get(DcMotor.class, "intakeMotor");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void spin(){
        intake1.setPower(0.75);
    }
}
