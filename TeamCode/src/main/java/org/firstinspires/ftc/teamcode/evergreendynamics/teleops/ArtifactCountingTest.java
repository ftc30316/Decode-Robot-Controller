package org.firstinspires.ftc.teamcode.evergreendynamics.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;

@TeleOp

public class ArtifactCountingTest extends LinearOpMode {
    private DistanceSensor firstArtifactSensor;
    private DistanceSensor secondArtifactSensor;
    private DistanceSensor thirdArtifactSensor;

    @Override
    public void runOpMode() {

        firstArtifactSensor = hardwareMap.get(DistanceSensor.class, "distancesensor1");
        secondArtifactSensor = hardwareMap.get(DistanceSensor.class, "distancesensor2");
        thirdArtifactSensor = hardwareMap.get(DistanceSensor.class, "distancesensor3");

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket distanceValues = new TelemetryPacket();
            distanceValues.put("ONE Distance detected: ", firstArtifactSensor.getDistance(DistanceUnit.INCH));
            distanceValues.put("TWO Distance detected: ", secondArtifactSensor.getDistance(DistanceUnit.INCH));
            distanceValues.put("THREE Distance detected: ", thirdArtifactSensor.getDistance(DistanceUnit.INCH));
            FtcDashboard.getInstance().sendTelemetryPacket(distanceValues);
            idle();
        }
    }
    public int getNumberOfArtifacts() {

        int numberOfArtifacts = 0;

        if (firstArtifactSensor.getDistance(DistanceUnit.INCH) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        if (secondArtifactSensor.getDistance(DistanceUnit.INCH) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        if (thirdArtifactSensor.getDistance(DistanceUnit.INCH) < InputValues.ARTIFACT_DISTANCE_DETECTION) {
            numberOfArtifacts++;
        }
        return numberOfArtifacts;
    }
}