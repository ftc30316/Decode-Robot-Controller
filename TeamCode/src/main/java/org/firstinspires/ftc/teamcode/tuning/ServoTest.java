package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp

public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");

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

//                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//                    double tx = result.getTx(); // How far left or right the target is (degrees)
//                    double ty = result.getTy(); // How far up or down the target is (degrees)
//                    double ta = result.getTa(); // How big the target looks (0%-100% of the image)
//
//                    telemetry.addData("Target X", tx);
//                    telemetry.addData("Target Y", ty);
//                    telemetry.addData("Target Area", ta);
//                } else {
//                    telemetry.addData("Limelight", "No Targets");
//                }

                //gives random error between -1 and 1; supposed to simulate limelight
                double error = (Math.random() * 2) - 1.0;
                //moves servo to counteract error
                servo1.setPosition(error * -1);

                Pose2d pose = drive.localizer.getPose();

                telemetry.addData("error: " , String.valueOf(error));
                telemetry.update();
                sleep(1000);

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
