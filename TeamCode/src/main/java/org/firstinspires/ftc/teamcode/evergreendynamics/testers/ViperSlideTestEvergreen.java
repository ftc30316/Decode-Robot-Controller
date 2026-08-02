package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
@TeleOp
public class ViperSlideTestEvergreen extends LinearOpMode {
    enum ViperSlideState {

        DOWN,

        UP
    }

    private int viperSlideRangeValidValue(int viperSlideUpTicks){
        //Minimum value for viper slide is 0 and maximum value is assumed to be 200
        if (viperSlideUpTicks < InputValues.VIPER_SLIDE_MIN_RANGE){
            viperSlideUpTicks = InputValues.VIPER_SLIDE_MIN_RANGE;
        }
        if (viperSlideUpTicks > InputValues.VIPER_SLIDE_MAX_RANGE){
            viperSlideUpTicks = InputValues.VIPER_SLIDE_MAX_RANGE;
        }
        return viperSlideUpTicks;
    }

    ViperSlideState viperSlideState = ViperSlideState.DOWN;
    @Override

    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(InputValues.VIPER_SLIDE_MIN_RANGE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.8);
        waitForStart();

        int viperSlideUpTicks = InputValues.VIPER_SLIDE_MAX_RANGE;

        while(opModeIsActive()){
            int currentTicks = motor.getCurrentPosition();
            telemetry.addData("Current Position Is: ", currentTicks);




            switch (viperSlideState) {
                case DOWN:
                    motor.setTargetPosition(InputValues.VIPER_SLIDE_MIN_RANGE);
                    motor.setPower(0.8);
                    if (gamepad1.crossWasPressed()) {
                        viperSlideState = ViperSlideState.UP;
                    }
                    break;
                case UP:
                    if (gamepad1.leftBumperWasPressed()) {
                        viperSlideUpTicks += InputValues.VIPER_SLIDE_STEP_SIZE;

                    }
                    if (gamepad1.rightBumperWasPressed()) {
                        viperSlideUpTicks -= InputValues.VIPER_SLIDE_STEP_SIZE;

                    }
                    viperSlideUpTicks = viperSlideRangeValidValue(viperSlideUpTicks);
                    motor.setTargetPosition(viperSlideUpTicks);
                    motor.setPower(0.8);
                    if (gamepad1.crossWasPressed()) {
                        viperSlideState = ViperSlideState.DOWN;
                    }
            }
            telemetry.addData("Change in ticks", viperSlideUpTicks - InputValues.VIPER_SLIDE_MAX_RANGE);
            telemetry.update();
        }

    }
}
