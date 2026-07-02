package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoEvergreenTest extends LinearOpMode {
    private Servo servoPort4;
    @Override

    public void runOpMode() throws InterruptedException {
        //Get the servo class
        servoPort4 = hardwareMap.get(Servo.class, "servoPort4");
        waitForStart();

        while (opModeIsActive()) {
            //turn it with joystick
            //servo expects a value between 0 and 1 but the joystick
            //ranges from -1 to 1
            float pos = (gamepad1.right_stick_x + 1) / 2.0f;
            telemetry.addData("Stick x position", pos);
            servoPort4.setPosition(pos);
            telemetry.update();

        }


    }
}
