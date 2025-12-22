package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Keybinds {

    public Gamepad gamepad1 = null;

    public Gamepad gamepad2 = null;

    public Keybinds (Gamepad gamepad1, Gamepad gamepad2) {

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

    }

    // Turret
    public boolean flywheelWasPressed() {
        gamepad1.circleWasPressed();
        return true;
    }
    public boolean liftWheelWasPressed() {
        gamepad1.crossWasPressed();
        return true;
    }
    public boolean turretLockingStateWasPressed() {
        gamepad1.dpadLeftWasPressed();
        return true;
    }




    public boolean turretManualVelocityIncreaseWasPressed() {
        gamepad1.dpadUpWasPressed();
        return true;
    }

    public boolean turretManualVelocityDecreaseWasPressed() {
        gamepad1.dpadUpWasPressed();
        return true;
    }

    // Mecanum drive
}
