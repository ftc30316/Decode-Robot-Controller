package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ColorTestEvergreen extends LinearOpMode {
    enum LightColor {
        RED,

        ORANGE,

        YELLOW,

        GREEN,

        BLUE,

        PURPLE

    }

    LightColor lightColor = LightColor.RED;

    @Override

    public void runOpMode() throws InterruptedException {
        Servo colorServo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while(opModeIsActive()){
            lightColor = GetLightColor();
            UpdateLightColor(lightColor);
            telemetry.addData("Color", lightColor);
            telemetry.update();

        }

    }

    private LightColor GetLightColor(){
        if (gamepad1.dpadUpWasPressed()){
            switch (lightColor){
                case RED: return LightColor.PURPLE;
                case ORANGE: return LightColor.RED;
                case YELLOW: return LightColor.ORANGE;
                case GREEN: return LightColor.YELLOW;
                case BLUE: return LightColor.GREEN;
                case PURPLE: return LightColor.BLUE;

            }
        } else if (gamepad1.dpadDownWasPressed()) {
            switch (lightColor) {
                case RED:
                    return LightColor.ORANGE;
                case ORANGE:
                    return LightColor.YELLOW;
                case YELLOW:
                    return LightColor.GREEN;
                case GREEN:
                    return LightColor.BLUE;
                case BLUE:
                    return LightColor.PURPLE;
                case PURPLE:
                    return LightColor.RED;
            }

        }
        return lightColor;

    }
    private void UpdateLightColor(LightColor newColor){

        switch (newColor){
            case RED:

                colorServo.setPosition(0.28);

                break;
            case ORANGE:

                colorServo.setPosition(0.31);
                break;
            case YELLOW:

                colorServo.setPosition(0.33);
                break;
            case GREEN:

                colorServo.setPosition(0.45);
                break;
            case BLUE:

                colorServo.setPosition(0.50);
                break;
            case PURPLE:

                colorServo.setPosition(0.55);
                break;
        }

    }
}
