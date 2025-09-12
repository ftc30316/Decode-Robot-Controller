package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AprilTagProcessor myAprilTagProcessor;
        // Create the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal myVisionPortal;

        // Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Leafy"), myAprilTagProcessor);

        // Enable or disable the AprilTag processor.
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                List<AprilTagDetection> myAprilTagDetections;  // list of all detections
                AprilTagDetection myAprilTagDetection;         // current detection in for() loop
                int myAprilTagIdCode;                           // ID code of current detection, in for() loop

                // Get a list of AprilTag detections.
                myAprilTagDetections = myAprilTagProcessor.getDetections();

                // Cycle through through the list and process each AprilTag.

                telemetry.addLine(String.valueOf(myAprilTagDetections.size()));

                for (int i = 0; i < myAprilTagDetections.size(); i++) {
                    myAprilTagDetection = myAprilTagDetections.get(i);

                    myAprilTagIdCode = myAprilTagDetection.id;

                    telemetry.addLine(String.valueOf(myAprilTagIdCode));

                    // Now take action based on this tag's ID code, or store info for later action.

                }

                telemetry.update();

            }
        } else{
            throw new RuntimeException();
        }
    }
}
