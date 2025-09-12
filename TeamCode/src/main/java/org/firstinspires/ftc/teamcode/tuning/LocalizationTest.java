package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //Servo servo1 = hardwareMap.get(Servo.class, "servo1");

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

               for(int i = 0; i < myAprilTagDetections.size(); i++) {
                   myAprilTagDetection = myAprilTagDetections.get(i);

                    myAprilTagIdCode = myAprilTagDetection.id;

                    telemetry.addLine(String.valueOf(myAprilTagIdCode));

                    // Now take action based on this tag's ID code, or store info for later action.

                }

                telemetry.update();
//LLResult result = limelight.getLatestResult();
//if (result != null && result.isValid()) {
//    double tx = result.getTx(); // How far left or right the target is (degrees)
//    double ty = result.getTy(); // How far up or down the target is (degrees)
//    double ta = result.getTa(); // How big the target looks (0%-100% of the image)
//
//    telemetry.addData("Target X", tx);
//    telemetry.addData("Target Y", ty);
//    telemetry.addData("Target Area", ta);
//} else {
//    telemetry.addData("Limelight", "No Targets");
//}
//                //gives random error between -1 and 1; supposed to simulate limelight
//                double error = (Math.random() * 2) - 1.0;
//                //moves servo to counteract error
//                servo1.setPosition(error * -1);
//
//                Pose2d pose = drive.localizer.getPose();
//
//                //when goal is positioned at 60,60, we get relative distance for x and y values
//                double xDistanceFromGoal = Math.abs(pose.position.x -  60);
//                double yDistanceFromGoal = Math.abs(pose.position.y -  60);
//
//                telemetry.addData("x", pose.position.x);
//                telemetry.addData("y", pose.position.y);
//                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//                telemetry.addData("distance from goal:" , String.valueOf(xDistanceFromGoal) + "," + String.valueOf(yDistanceFromGoal));
//                telemetry.addData("error: " , String.valueOf(error));
//                telemetry.update();
//                sleep(1000);

//                TelemetryPacket packet = new TelemetryPacket();
//                packet.fieldOverlay().setStroke("#3F51B5");
//                Drawing.drawRobot(packet.fieldOverlay(), pose);
//                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }



        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();


                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
