package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc
import java.util.List;
@TeleOp
public class AprilTagTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AprilTagProcessor myAprilTagProcessor;
        // Create the AprilTag processor and assign it to a variable.
        //myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessor = myAprilTagProcessorBuilder
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(getDecodeTagLibrary())
                .build();

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


                telemetry.addLine(String.valueOf(myAprilTagDetections.size()));

                for (int i = 0; i < myAprilTagDetections.size(); i++) {
                    myAprilTagDetection = myAprilTagDetections.get(i);
                    double myTagPoseRange = myAprilTagDetection.ftcPose.range;
                    double myTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
                    double myTagPoseElevation = myAprilTagDetection.ftcPose.elevation;

                    myAprilTagIdCode = myAprilTagDetection.id;

                    telemetry.addLine(String.valueOf(myAprilTagIdCode));
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("range", myAprilTagDetection.ftcPose.range); // distance from camera to target
                    packet.put("bearing", myAprilTagDetection.ftcPose.bearing); // angle the camera must turn (left/right) to face target
                    packet.put("elevation", myAprilTagDetection.ftcPose.elevation); // angle the camera must tilt (up/down) to face target
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    // Now take action based on this tag's ID code, or store info for later action.


                }

                telemetry.update();

            }
        } else{
            throw new RuntimeException();
        }


    }

    public static AprilTagLibrary getDecodeTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(20, "BlueGoal",
                        6, DistanceUnit.INCH)
                .addTag(24, "RedGoal",
                        6, DistanceUnit.INCH)
                .addTag(21, "GPP",
                        6, DistanceUnit.INCH)
                .addTag(22, "PGP",
                        6, DistanceUnit.INCH)
                .addTag(23, "PPG",
                        6, DistanceUnit.INCH)
                .build();
    }

}
