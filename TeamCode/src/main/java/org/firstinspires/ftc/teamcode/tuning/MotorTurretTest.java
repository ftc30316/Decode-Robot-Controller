package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class MotorTurretTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
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
            //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            // Reset the encoder during initialization
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Set the amount of ticks to move
            //turretMotor.setTargetPosition((int) (90.0 * 5.3277777778));
            turretMotor.setTargetPosition(0);

            // Switch to RUN_TO_POSITION mode
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start the motor moving by setting the max velocity to ___ ticks per second
            turretMotor.setVelocity(2781.1);

            waitForStart();
            while (opModeIsActive()) {
//                drive.setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(
//                                -gamepad1.left_stick_y,
//                                -gamepad1.left_stick_x
//                        ),
//                        -gamepad1.right_stick_x
//                ));
//
//                drive.updatePoseEstimate();

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


                    // Now take action based on this tag's ID code, or store info for later action.

                    double currentPosition = turretMotor.getCurrentPosition();
                    double motorTicks = myAprilTagDetection.ftcPose.bearing * 5.3277777778;
                    double newPosition = currentPosition - motorTicks;

                    turretMotor.setTargetPosition((int) newPosition);
//                    turretMotor.setVelocity(myTagPoseBearing);

                    telemetry.addLine(String.valueOf(myAprilTagIdCode));
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("range", myAprilTagDetection.ftcPose.range); // distance from camera to target
                    packet.put("bearing", myAprilTagDetection.ftcPose.bearing); // angle the camera must turn (left/right) to face target
                    packet.put("elevation", myAprilTagDetection.ftcPose.elevation); // angle the camera must tilt (up/down) to face target
//                    packet.put("current position", currentPosition);
//                    packet.put("motor ticks", motorTicks);
//                    packet.put("new position", newPosition);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    packet = new TelemetryPacket();
                    packet.put("velocity", turretMotor.getVelocity());
                    packet.put("position", turretMotor.getCurrentPosition());
                    packet.put("is at target", !turretMotor.isBusy());
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    sleep(25);

//                    Pose2d pose = drive.localizer.getPose();
//
//                    telemetry.addData("error: " , String.valueOf(error));
//                    telemetry.update();
//                    sleep(1000);


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
