package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp
@Disabled

public class DriveTurretTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
//        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            // Reset the encoder during initialization
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            waitForStart();
            double ticksPerRev = turretMotor.getMotorType().getTicksPerRev();
            //turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("velocity", turretMotor.getVelocity());
            packet.put("position", turretMotor.getCurrentPosition());
            packet.put("is at target", !turretMotor.isBusy());
            packet.put("ticks per rev", ticksPerRev);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            sleep(5000);

            //Set the amount of ticks to move
            turretMotor.setTargetPosition(100);

            // Switch to RUN_TO_POSITION mode
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start the motor moving by setting the max velocity to ___ ticks per second
            turretMotor.setVelocity(10);




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
                packet.put("ticks per rev", ticksPerRev);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();
                sleep(50);
            }
        }
        else {
        throw new RuntimeException();
            }
        }
    }
