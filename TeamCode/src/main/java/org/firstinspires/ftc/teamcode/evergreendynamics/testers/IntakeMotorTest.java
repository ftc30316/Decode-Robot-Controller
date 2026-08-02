package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
enum MotorState{
    OFF,
    ON
}

enum MotorDirection{
    FORWARD,
    BACKWARD
}
public class IntakeMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
        waitForStart();

        MotorDirection motorDirection = MotorDirection.FORWARD;
        MotorState motorState = MotorState.OFF;

        while(opModeIsActive()){
            switch (motorState) {
                case OFF:
                    motor.setPower(0);
                    break;
                case ON:
                    motor.setPower(0.8);
                    //TODO: determine direction and run motor in that direction
                    break;

            }
        }
    }

}
