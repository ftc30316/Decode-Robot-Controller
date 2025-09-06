package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();
            Vector2d strafeVector = new Vector2d(24, 12);
            Actions.runBlocking(
                //drive.actionBuilder(beginPose).splineTo(new Vector2d(24, 24), Math.PI / 2)
                    drive.actionBuilder(beginPose).setTangent(0)
                            //.splineToConstantHeading(new Vector2d(24, 24),  Math.toRadians(180))
                          //  .splineToConstantHeading(new Vector2d())
                            .strafeTo(new Vector2d(24, 24))
                            .strafeTo(new Vector2d(48, 24))
                            .strafeTo(new Vector2d(72, 0))
                        .build());

            drive.updatePoseEstimate();
            Pose2d position = drive.localizer.getPose();

            telemetry.addLine(position.toString());
            sleep(5000);
        }
    }
}