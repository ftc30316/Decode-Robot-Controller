package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

@TeleOp (group = "Evergreen Testing")
public class FlywheelVelocityTest extends LinearOpMode {
    public DouglasFIRST douglasFIRST;

    @Override
    public void runOpMode() {
        try {
            Pose2d startPose = new Pose2d (0, 0, 0);
            double turretStartHeading = Math.toDegrees(startPose.heading.toDouble()); //PoseStorage.loadTurretHeading(hardwareMap.appContext);

            // 1. start robot at 0, 0, 0. turret will lock on blue goal.
            // 2. driver reports pose information to data recorder
            // 3. driver shoots artifacts at blue goal with different velocities & reports velocities that work to the data recorder
            // 4. driver changes turret position to red goal
            // 5. driver repeats step 3 for red
            // 6. driver moves robot to different location
            // 7. driver repeats steps 2 - 5



            telemetry.addLine("The flywheel velocity changes in increments of 5. To increase, press up on the dpad. To decrease, press down on the dpad.");

            this.douglasFIRST = new DouglasFIRST(hardwareMap, gamepad1, gamepad2, telemetry, startPose, DouglasFIRST.DriveMode.ROBOT_CENTRIC, Turret.TurretVelocityMode.MANUAL); //startPose);


            waitForStart();

            douglasFIRST.start(startPose.heading.toDouble(), 0); // turret heading is robot centric

            // Sets up the driving system
            while (opModeIsActive()) {
                douglasFIRST.loop();

                telemetry.update();
            }

        } catch (Exception e) {
            e.printStackTrace();
        } finally {

        }

    }
}
