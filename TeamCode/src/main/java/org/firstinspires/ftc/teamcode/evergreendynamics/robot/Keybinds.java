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
        return gamepad1.circleWasPressed();
    }
    public boolean liftWheelWasPressed() {
        return gamepad1.crossWasPressed();
    }
    public boolean turretLockingStateWasPressed() {
        return gamepad1.dpadLeftWasPressed();
    }
    public boolean turretManualAdjustmentLeftBumperIsPressed() {
        return gamepad2.left_bumper;
    }
    public boolean turretManualAdjustmentRightBumperIsPressed() {
        return gamepad2.right_bumper;
    }
    public float turretManualAdjustmentLeftJoystickX() {
        return gamepad2.left_stick_x;
    }
    public boolean turretManualVelocityIncreaseWasPressed() {
        return gamepad1.dpadUpWasPressed();
    }
    public boolean turretManualVelocityDecreaseWasPressed() {
        return gamepad1.dpadUpWasPressed();
    }

    // Intake
    public boolean changeIntakeState() {
        return gamepad1.squareWasPressed();
    }

    // Mecanum drive
    public boolean changeDrivePowersRightWasPressed() {
        return gamepad1.dpadRightWasPressed();
    }
}
