package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp

public class DistanceSensorTest extends LinearOpMode {
    private DistanceSensor artifactSlot1;
    private DistanceSensor artifactSlot2;
    private DistanceSensor artifactSlot3;

    @Override
    public void runOpMode() {
        artifactSlot1 = hardwareMap.get(DistanceSensor.class, "distancesensor1");
        artifactSlot2 = hardwareMap.get(DistanceSensor.class, "distancesensor2");
        artifactSlot3 = hardwareMap.get(DistanceSensor.class, "distancesensor3");


        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket distanceValues = new TelemetryPacket();
            distanceValues.put("ONE Distance detected: ", artifactSlot1.getDistance(DistanceUnit.INCH));
            distanceValues.put("TWO Distance detected: ", artifactSlot2.getDistance(DistanceUnit.INCH));
            distanceValues.put("THREE Distance detected: ", artifactSlot3.getDistance(DistanceUnit.INCH));
            FtcDashboard.getInstance().sendTelemetryPacket(distanceValues);
            idle();
        }
    }
}