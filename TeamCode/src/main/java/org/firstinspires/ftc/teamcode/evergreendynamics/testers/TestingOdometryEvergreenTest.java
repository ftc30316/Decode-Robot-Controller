package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Keybinds;
@TeleOp
public class TestingOdometryEvergreenTest extends LinearOpMode  {
    private MecanumDrive mecanumDrive;
    @Override
    public void runOpMode() throws InterruptedException {
        Keybinds keybinds = new Keybinds(this.gamepad1, this.gamepad2);
        Pose2d beginPose = new Pose2d(0,0,0);
        this.mecanumDrive = new MecanumDrive(hardwareMap, keybinds, beginPose);
        waitForStart();

        while (opModeIsActive()) {
            this.mecanumDrive.updatePoseEstimate();
            Pose2d currentPosition = this.mecanumDrive.localizer.getPose();
            telemetry.addData("The x position is: ", currentPosition.position.x);
            telemetry.addData("The y position is: ", currentPosition.position.y);
            telemetry.addData("The heading is: ", currentPosition.heading);
            telemetry.update();

            TelemetryPacket robotPose = new TelemetryPacket();
            robotPose.put("The x position is: ", currentPosition.position.x);
            robotPose.put("The y position is", currentPosition.position.y);
            robotPose.put("The heading is", currentPosition.heading);
            FtcDashboard.getInstance().sendTelemetryPacket(robotPose);

        }


    }
}
