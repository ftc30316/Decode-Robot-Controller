package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
    public MecanumDrive mecanumDrive;


    public DouglasFIRST(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Pose2d beginPose) {
        this.mecanumDrive = new MecanumDrive(hardwareMap, gamepad1, beginPose);
        this.intake = new Intake(hardwareMap, gamepad1, gamepad2, telemetry);

        this.turret = new Turret(hardwareMap, telemetry, gamepad1, gamepad2, getGoalPosition(hardwareMap), mecanumDrive, intake);

        telemetry.update();
    }

    public void start(double robotHeadingDeg, double turretStartHeadingDeg) {
        turret.initialize(robotHeadingDeg, turretStartHeadingDeg);

        //Creates background thread
        turret.createTurretBackgroundThread();
        // Intake motor starts, flywheel starts, turret starts looking for the BLUE goal
        turret.turretBackgroundThread.start();
    }

    public void loop() {
        if (mecanumDrive.drivePowers == MecanumDrive.DrivePowers.SLOW) {
            telemetry.addData("robot speed", "SLOW");
            mecanumDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.5,
                            -gamepad1.left_stick_x * 0.5
                    ),
                    -gamepad1.right_stick_x * 0.5
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
        mecanumDrive.loop();
    }

    public void shutdown() {
        turret.stopTurretBackgroundThread();
    }

    public Vector2d getGoalPosition(HardwareMap hardwareMap) {
        Vector2d goalPosition = InputValues.BLUE_GOAL_POSITION;
        NormalizedColorSensor signColorSensor = hardwareMap.get(NormalizedColorSensor.class, "signColorSensor");

        NormalizedRGBA signColor = signColorSensor.getNormalizedColors();
        float[] hsv = new float[3];

        Color.colorToHSV(signColor.toColor(), hsv);

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (hue < 20 || hue > 340) { // if hue is in red range
            goalPosition = InputValues.RED_GOAL_POSITION;
        }

        return goalPosition;
    }

}
