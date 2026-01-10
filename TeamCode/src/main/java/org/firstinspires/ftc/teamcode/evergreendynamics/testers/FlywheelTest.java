package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled

public class FlywheelTest extends LinearOpMode {
    private DcMotor flywheel1;
    private DcMotor flywheel2;

    @Override
    public void runOpMode() {
        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");

        flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y * 0.6;

            flywheel1.setPower(power);
            flywheel2.setPower(power);

            telemetry.addData("Power:", power);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Power:", power);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            idle();
        }
    }

}