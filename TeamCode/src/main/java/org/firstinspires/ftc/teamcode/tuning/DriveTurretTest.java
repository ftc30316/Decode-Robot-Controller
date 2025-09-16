package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp

public class DriveTurretTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            // Reset the encoder during initialization
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

                Pose2d pose = drive.localizer.getPose();

                leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Set the amount of ticks to move
                leftBack.setTargetPosition(250);

                // Switch to RUN_TO_POSITION mode
                leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Start the motor moving by setting the max velocity to ___ ticks per second
                leftBack.setVelocity(1000);

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("velocity", leftBack.getVelocity());
                packet.put("position", leftBack.getCurrentPosition());
                packet.put("is at target", !leftBack.isBusy());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();
                sleep(100);
            }
        }
        else {
        throw new RuntimeException();
            }
        }
    }
