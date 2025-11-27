package org.firstinspires.ftc.teamcode.evergreendynamics;

public class Helper {

    public static void sleep (int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
