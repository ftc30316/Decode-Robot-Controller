package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Helper;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;

@TeleOp

public class ArtifactCountingTest extends LinearOpMode {
    private DistanceSensor frontArtifactSensor;
    private DistanceSensor middleArtifactSensor;
    private DistanceSensor backArtifactSensor;

    @Override
    public void runOpMode() {

        frontArtifactSensor = hardwareMap.get(DistanceSensor.class, "frontArtifactSensor");
        middleArtifactSensor = hardwareMap.get(DistanceSensor.class, "middleArtifactSensor");
        backArtifactSensor = hardwareMap.get(DistanceSensor.class, "backArtifactSensor");

        waitForStart();

        while (opModeIsActive()) {

            double frontSensorDetection = getAverageDistance(frontArtifactSensor);
            double middleSensorDetection = getAverageDistance(middleArtifactSensor);
            double backSensorDetection = getAverageDistance(backArtifactSensor);

            TelemetryPacket distanceValues = new TelemetryPacket();
            distanceValues.put("FRONT Distance detected: ", frontSensorDetection);
            distanceValues.put("MIDDLE Distance detected: ", middleSensorDetection);
            distanceValues.put("BACK Distance detected: ", backSensorDetection);
            distanceValues.put("NUM OF ARTIFACTS: ", getNumberOfArtifacts());
            distanceValues.put("Artifact detecting distance is ", InputValues.ARTIFACT_DISTANCE_DETECTION);

            if (frontSensorDetection < InputValues.ARTIFACT_DISTANCE_DETECTION) {
                distanceValues.addLine("FRONT IS DETECTING");
                telemetry.addLine("FRONT IS DETECTING");
            }
            if (middleSensorDetection < InputValues.ARTIFACT_DISTANCE_DETECTION) {
                distanceValues.addLine("MIDDLE IS DETECTING");
                telemetry.addLine("MIDDLE IS DETECTING");
            }
            if (backSensorDetection < InputValues.ARTIFACT_DISTANCE_DETECTION) {
                distanceValues.addLine("BACK IS DETECTING");
                telemetry.addLine("BACK IS DETECTING");
            }

            FtcDashboard.getInstance().sendTelemetryPacket(distanceValues);

            telemetry.addData("Number of Artifacts: ", getNumberOfArtifacts());

            telemetry.update();
            idle();
        }
    }
    public int getNumberOfArtifacts() {

        int numberOfArtifacts = 0;

        if (frontArtifactSensor.getDistance(DistanceUnit.INCH) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        if (middleArtifactSensor.getDistance(DistanceUnit.INCH) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        if (backArtifactSensor.getDistance(DistanceUnit.INCH) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        return numberOfArtifacts;
    }

    public double getAverageDistance(DistanceSensor distanceSensor) {

        double sum = 0;

        for(int i = 0; i < 7; i++) {
            double detection = distanceSensor.getDistance(DistanceUnit.INCH);
            Helper.sleep(5);
            sum = sum + detection;
        }

        return sum / 7;
    }
}