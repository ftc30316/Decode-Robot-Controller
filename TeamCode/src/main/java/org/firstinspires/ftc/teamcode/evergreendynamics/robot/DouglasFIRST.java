package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DouglasFIRST {
    public Intake intake;
    public Turret turret;
    public Telemetry telemetry;
    public MecanumDrive mecanumDrive;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public HardwareMap hardwareMap;
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public enum Alliance {
        BLUE,
        RED
    }
    DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    Alliance alliance = Alliance.BLUE;

    public DouglasFIRST(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Pose2d beginPose, DriveMode driveMode) {
        this.mecanumDrive = new MecanumDrive(hardwareMap, gamepad1, gamepad2, beginPose);
        this.intake = new Intake(hardwareMap, gamepad1, gamepad2, telemetry);
        this.telemetry = telemetry;
        this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, getGoalPosition(hardwareMap), mecanumDrive, intake);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.driveMode = driveMode;
    }

    public DouglasFIRST(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Vector2d goalPosition, Pose2d beginPose, DriveMode driveMode) {
        this.mecanumDrive = new MecanumDrive(hardwareMap, gamepad1, gamepad2, beginPose);
        this.intake = new Intake(hardwareMap, gamepad1, gamepad2, telemetry);
        this.telemetry = telemetry;
        this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, goalPosition, mecanumDrive, intake);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.driveMode = driveMode;
    }

    public void start(double robotHeadingDeg, double turretStartHeadingDeg) {
        turret.initialize(robotHeadingDeg, turretStartHeadingDeg);

        //Creates background thread
        turret.createTurretBackgroundThread();
        // Intake motor starts, flywheel starts, turret starts looking for the BLUE goal
        turret.turretBackgroundThread.start();
        // Turns on intake
        intake.turnOnIntake();
    }

    public void shootArtifacts() {
        turret.shoot();
    }

    public void loop() {
        switch (driveMode) {
            case ROBOT_CENTRIC:
                setRobotCentricDrivePowers();

                if (gamepad2.xWasPressed()) {
                    driveMode = DriveMode.FIELD_CENTRIC;
                }
                break;

            case FIELD_CENTRIC:
                setFieldCentricDrivePowers();

                if (gamepad2.xWasPressed()) {
                    driveMode = DriveMode.ROBOT_CENTRIC;
                }
                break;
        }

        mecanumDrive.loop();
        intake.loop();
        turret.loop();
    }

    public void setRobotCentricDrivePowers() {
        if (mecanumDrive.drivePowers == MecanumDrive.DrivePowers.SLOW) {
            telemetry.addData("robot speed", "SLOW");
            mecanumDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.25,
                            -gamepad1.left_stick_x * 0.25
                    ),
                    -gamepad1.right_stick_x * 0.25
            ));

        } else {
            telemetry.addData("robot speed", "FAST");
            mecanumDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }
    }

    public void setFieldCentricDrivePowers() {
        // Read pose
        mecanumDrive.updatePoseEstimate();

        Rotation2d negativeHeading = new Rotation2d(-mecanumDrive.localizer.getPose().heading.toDouble(), 0);

    // Create a vector from the gamepad x/y inputs
    // Then, rotate that vector by the inverse of that heading
        Vector2d input = negativeHeading.times(new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x));

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately
        mecanumDrive.setDrivePowers(
                new PoseVelocity2d(
                        input,
                        -gamepad1.right_stick_x
                )
        );

    }

    public void shutdown() {
        turret.stopTurretBackgroundThread();
    }

    public Vector2d getGoalPosition(HardwareMap hardwareMap) {
        Vector2d goalPosition = InputValues.BLUE_GOAL_POSITION;
        mecanumDrive.updatePoseEstimate();

        if (mecanumDrive.localizer.getPose().position.y < 0) {
            alliance = Alliance.BLUE;
            return goalPosition;
        }
        if (mecanumDrive.localizer.getPose().position.y > 0) {
            goalPosition = InputValues.RED_GOAL_POSITION;
            alliance = Alliance.RED;
            return goalPosition;
        }

        return goalPosition;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public TrajectoryActionBuilder getActionBuilder(Pose2d beginPose) {
        return mecanumDrive.actionBuilder(beginPose);
    }

    public TrajectoryActionBuilder getActionBuilder() {
        return mecanumDrive.actionBuilder(getCurrentPose());
    }

    public Pose2d getCurrentPose() {
        mecanumDrive.updatePoseEstimate();
        return mecanumDrive.localizer.getPose();
    }

    public void savePose() {
        mecanumDrive.updatePoseEstimate();
        PoseStorage.savePose(hardwareMap.appContext, mecanumDrive.localizer.getPose(), turret.getTurretDegrees());
    }

}
