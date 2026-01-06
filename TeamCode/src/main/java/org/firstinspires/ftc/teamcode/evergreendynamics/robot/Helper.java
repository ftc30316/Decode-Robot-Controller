package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Helper {

    public static void sleep (int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public static double getAverageDistance(DistanceSensor distanceSensor) {
        return distanceSensor.getDistance(DistanceUnit.INCH);

//        double sum = 0;
//
//        for(int i = 0; i < 7; i++) {
//            double detection = distanceSensor.getDistance(DistanceUnit.INCH);
//            Helper.sleep(5);
//            sum = sum + detection;
//        }
//
//        return sum / 7;
    }
}
