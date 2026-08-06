package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
enum MotorState{
    OFF,
    ON
}

enum MotorDirection{
    FORWARD,
    BACKWARD
}
@TeleOp
public class IntakeMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(1);
        motor.setVelocity(0);
        waitForStart();

        MotorDirection motorDirection = MotorDirection.FORWARD;
        MotorState motorState = MotorState.OFF;

        while(opModeIsActive()){
            double currentVelocity = motor.getVelocity();
            telemetry.addData("Current Velocity Is: ", currentVelocity);

            switch (motorState) {
                case OFF:
                    motor.setVelocity(0);

                    if (gamepad1.crossWasPressed()) {
                        motorState = MotorState.ON;
                    }
                    break;
                case ON:
                    motor.setVelocity(1000);

                    //TODO: determine direction and run motor in that direction
                    if (gamepad1.crossWasPressed()) {
                        motorState = MotorState.OFF;
                    }
                    break;
            }

            switch (motorDirection) {
                case FORWARD:
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                    if (gamepad1.dpadDownWasPressed()) {
                        motorDirection = MotorDirection.BACKWARD;
                    }

                    break;

                case BACKWARD:
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                    if (gamepad1.dpadUpWasPressed()) {
                        motorDirection = MotorDirection.FORWARD;
                    }
                    break;
            }
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Current Velocity", currentVelocity);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

}
