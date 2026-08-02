package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class ViperSlideTestEvergreen extends LinearOpMode {
    enum ViperSlideState {

        DOWN,

        UP
    }

    private int viperSlideRangeValidValue(int viperSlideUpTicks){
        //Minimum value for viper slide is 0 and maximum value is assumed to be 200
        if (viperSlideUpTicks < 0){
            viperSlideUpTicks = 0;
        }
        if (viperSlideUpTicks > 200){
            viperSlideUpTicks = 200;
        }
        return viperSlideUpTicks;
    }

    ViperSlideState viperSlideState = ViperSlideState.DOWN;
    @Override

    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.8);
        waitForStart();

        int viperSlideUpTicks = 200;

        while(opModeIsActive()){
            int currentTicks = motor.getCurrentPosition();
            telemetry.addData("Current Position Is: ", currentTicks);




            switch (viperSlideState) {
                case DOWN:
                    motor.setTargetPosition(0);
                    motor.setPower(0.8);
                    if (gamepad1.crossWasPressed()) {
                        viperSlideState = ViperSlideState.UP;
                    }
                    break;
                case UP:
                    if (gamepad1.leftBumperWasPressed()) {
                        viperSlideUpTicks += 10;

                    }
                    if (gamepad1.rightBumperWasPressed()) {
                        viperSlideUpTicks -= 10;

                    }
                    viperSlideUpTicks = viperSlideRangeValidValue(viperSlideUpTicks);
                    motor.setTargetPosition(viperSlideUpTicks);
                    motor.setPower(0.8);
                    if (gamepad1.crossWasPressed()) {
                        viperSlideState = ViperSlideState.DOWN;
                    }
            }
            telemetry.addData("Change in ticks", viperSlideUpTicks - 200);
            telemetry.update();
        }

    }
}
