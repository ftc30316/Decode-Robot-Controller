package org.firstinspires.ftc.teamcode.biobuzz;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@TeleOp (group = "Evergreen Teleop")
public class BiobuzzTeleOp extends LinearOpMode {
    public Robot robot;

    @Override
    public void runOpMode() {
        try {
            Pose2d startPose = PoseStorage.loadPose(hardwareMap.appContext);

            this.robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, startPose);

            //            telemetry.addData("auto end pose x", startPose.position.x);
//            telemetry.addData("auto end pose y", startPose.position.y);
//            telemetry.addData("auto end pose heading", Math.toDegrees(startPose.heading.toDouble()));
//            telemetry.addData("auto end turret heading", turretStartHeadingDeg);

            waitForStart();

            this.robot.init();

            // Sets up the driving system
            while (opModeIsActive()) {
                telemetry.clearAll();
                this.robot.loop();

                telemetry.update();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {

        }

    }
}
