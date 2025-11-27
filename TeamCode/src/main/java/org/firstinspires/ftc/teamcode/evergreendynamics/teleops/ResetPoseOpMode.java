package org.firstinspires.ftc.teamcode.evergreendynamics.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.PoseStorage;

@TeleOp (group = "Evergreen Testing")
public class ResetPoseOpMode extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        double turretStartingDegrees = -90;
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));

        PoseStorage.savePose(hardwareMap.appContext, beginPose, turretStartingDegrees);

        waitForStart();

    }
}
