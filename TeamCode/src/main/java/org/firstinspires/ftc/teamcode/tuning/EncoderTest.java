package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp

public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            // Reset the encoder during initialization
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            waitForStart();
            double ticksPerRev = turretMotor.getMotorType().getTicksPerRev();
            //leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("velocity", turretMotor.getVelocity());
            packet.put("position", turretMotor.getCurrentPosition());
            packet.put("is at target", !turretMotor.isBusy());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

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
//
//                Pose2d pose = drive.localizer.getPose();

                packet = new TelemetryPacket();
                packet.put("velocity", turretMotor.getVelocity());
                packet.put("position", turretMotor.getCurrentPosition());
                packet.put("is at target", !turretMotor.isBusy());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();
                sleep(200);
            }
        }
        else {
        throw new RuntimeException();
            }
        }
    }
