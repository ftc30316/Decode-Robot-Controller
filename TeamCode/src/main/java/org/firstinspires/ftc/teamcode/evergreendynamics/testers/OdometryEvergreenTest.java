package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Keybinds;
@TeleOp
public class OdometryEvergreenTest extends LinearOpMode {
    private MecanumDrive mecanumDrive;

    @Override

    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        Keybinds keybinds = new Keybinds(this.gamepad1, this.gamepad2);
        this.mecanumDrive = new MecanumDrive(hardwareMap, keybinds, beginPose);
        waitForStart();

        while (opModeIsActive()) {
            this.mecanumDrive.updatePoseEstimate();
            Pose2d currentPosition = this.mecanumDrive.localizer.getPose();
            telemetry.addData("Robot x position", currentPosition.position.x);
            telemetry.addData("Robot y position", currentPosition.position.y);
            telemetry.addData("Robot heading", currentPosition.heading.toDouble());
            double distanceFromStart = Math.sqrt(
                    Math.pow(currentPosition.position.x, 2) +
                            Math.pow(currentPosition.position.y, 2));

            telemetry.addData("Distance from start", distanceFromStart + "in");
            telemetry.update();

            TelemetryPacket telemetryPacket = new TelemetryPacket();
            telemetryPacket.put("Robot x position", currentPosition.position.x);
            telemetryPacket.put("Robot y position", currentPosition.position.y);
            telemetryPacket.put("Robot heading", currentPosition.heading.toDouble());
            telemetryPacket.put("Distance from start", distanceFromStart + "in");
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);


        }
    }
}