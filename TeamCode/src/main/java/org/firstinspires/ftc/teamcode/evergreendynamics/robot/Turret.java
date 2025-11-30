package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Turret {
    // Setting up the state machines for the two states, aiming the turret towards the goal and shooting the artifacts to score

    public enum Scoring {
        SEARCHING,
        HOLDING,
        SHOOTING,
        RESETTING
    }
    public enum FlywheelState {
        ON,

        REVERSE
    }
    public enum TurretLockingState {
        AUTO,
        MANUAL
    }
    FlywheelState flywheelState = FlywheelState.ON;
    TurretLockingState turretLockingState = TurretLockingState.AUTO;

    public Scoring turretScoring = Turret.Scoring.SEARCHING;
    private Telemetry telemetry;

    public volatile Gamepad gamepad1 = null;

    public volatile Gamepad gamepad2 = null;
    DcMotorEx turretMotor;
    private DcMotorEx leftShootingFlywheel;
    private DcMotorEx rightShootingFlywheel;
    private CRServo leftLiftFlywheel;
    private CRServo rightLiftFlywheel;
    private MecanumDrive mecanumDrive;
    public Vector2d goalPosition;

    public Thread turretBackgroundThread;
    public int flywheelSpeed = 2000;
    private volatile boolean runAutoAimThread = true;
    private volatile double turretDegrees = 0;
    private double turretZeroRelRobotDeg;
    double desiredFieldAngleDeg;

    public ElapsedTime liftTimer = new ElapsedTime();

    public Turret(HardwareMap hardwareMap, Telemetry telemetry,
                  Gamepad gamepad1, Gamepad gamepad2,
                  Vector2d goalPosition,
                  MecanumDrive mecanumDrive) {

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.goalPosition = goalPosition;
        this.mecanumDrive = mecanumDrive;

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftShootingFlywheel = hardwareMap.get(DcMotorEx.class, "leftShootingFlywheel");
//        rightShootingFlywheel = hardwareMap.get(DcMotorEx.class, "rightShootingFlywheel");
//        leftLiftFlywheel = hardwareMap.get(CRServo.class, "leftLiftFlywheel");
//        rightLiftFlywheel = hardwareMap.get(CRServo.class, "leftLiftFlywheel");
//        //liftSensor = hardwareMap.get(DistanceSensor.class, "distancesensor1");
//
//        leftShootingFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightShootingFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        leftShootingFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightShootingFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        leftLiftFlywheel.setDirection(CRServo.Direction.REVERSE);
//        rightLiftFlywheel.setDirection(CRServo.Direction.FORWARD);

    }

    public void initialize(double robotHeadingStartDeg,
                           double turretFieldAngleStartDeg) {

        // 1) Reset encoder so current mechanical position = 0 ticks
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 2) Compute turret angle relative to robot body when encoder = 0
        //    K = (turret field angle) - (robot field heading)
        //    This is a constant relationship that we use later.
        turretZeroRelRobotDeg = turretFieldAngleStartDeg - robotHeadingStartDeg;
    }

    public void createTurretBackgroundThread() {
        // Creates a background thread so that while the robot is driving, intaking, and sorting, the turret can always be auto-locked on the goal
        this.turretBackgroundThread = new Thread(new AutoAimThread());
    }

    public void stopTurretBackgroundThread() {
        runAutoAimThread = false;
    }

    class AutoAimThread implements Runnable {
        @Override
        public void run() {
            try {
                while (runAutoAimThread) {
                    adjustTurret();
                    Helper.sleep(InputValues.TURRET_THREAD_SLEEP_TIME_MILLIS);
                }
            } catch (Exception e) {

            } finally {
                turretMotor.setPower(0);
            }
        }
    }

    public void turretControl() {
        telemetry.addData("Turret locking state: ", turretLockingState);
        switch (turretLockingState) {
            case AUTO:
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                adjustTurret();
                if (gamepad1.dpad_left) {
                    turretLockingState = TurretLockingState.MANUAL;
                }
                break;
            case MANUAL:
                stopTurretBackgroundThread();
                turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (gamepad2.left_bumper) {
                    turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    turretMotor.setPower(InputValues.FLYWHEEL_SLOPE);
                }
                else if (gamepad2.right_bumper) {
                    turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    turretMotor.setPower(InputValues.FLYWHEEL_SLOPE);
                }
                else if (gamepad2.left_stick_x != 0) {
                    double turretJoystickPower = gamepad2.left_stick_x;
                    turretMotor.setPower(turretJoystickPower);
                }
                else {
                    turretMotor.setPower(0);
                }
                break;
        }
    }

    // Starts the flywheel
    public void triggerFlywheel() {
        telemetry.addData("Flywheel state is: ", flywheelState);
        switch (flywheelState) {
            case ON:
                leftShootingFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                rightShootingFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                leftShootingFlywheel.setVelocity(flywheelSpeed);
                rightShootingFlywheel.setVelocity(flywheelSpeed);
                if (gamepad1.dpadUpWasPressed()) {
                    flywheelSpeed += 25;
                }
                if (gamepad1.dpadDownWasPressed()) {
                    flywheelSpeed -= 25;
                }
                if (gamepad1.circleWasPressed()) {//gamepad2.right_trigger > 0.5) {
                    flywheelState = FlywheelState.REVERSE;
                }
                break;
            case REVERSE:
//                leftShootingFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
//                rightShootingFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
//                leftShootingFlywheel.setVelocity(0);
//                rightShootingFlywheel.setVelocity(0);
                if (gamepad1.circleWasPressed()) {//gamepad2.left_trigger > 0.5) {
                    flywheelState = FlywheelState.ON;
                }
                break;
        }
    }

    // A state machine that checks if there is an artifact in the lift slot, and moves artifact into flywheel when x is pressed
    public void score() {
        telemetry.addData("score state", turretScoring);
        mecanumDrive.updatePoseEstimate();
        Pose2d robotPose = mecanumDrive.localizer.getPose();
        double robotX = robotPose.position.x;
        double robotY = robotPose.position.y;

        // get goal position; blue: -66, -66, red: -66, 66
        double goalX = goalPosition.x;
        double goalY = goalPosition.y;

        // find A: (Yr - Yb)
        double A = robotY - goalY;

        // find C: (Xr - Xb)
        double C = robotX - goalX;

        // find distance (D): sqrt(A^2 + C^2)
        double D = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
        telemetry.addData("Distance from goal is:", D);
        telemetry.addData("Flywheel speed is: ", flywheelSpeed);
        telemetry.addData("Shooting state is: ", turretScoring);
        switch (turretScoring) {
            case SEARCHING:
//                leftLiftFlywheel.setPower(0);
//                rightLiftFlywheel.setPower(0);
                //if distance sensed is less than __, transition to holding
                turretScoring = Scoring.HOLDING;
                break;
            case HOLDING:
                if (gamepad1.cross) {
                    liftTimer.reset();
                    turretScoring = Scoring.SHOOTING;
                }
                break;
            case SHOOTING:
////                leftLiftFlywheel.setPower(1.0);
////                rightLiftFlywheel.setPower(1.0);
//                if (liftTimer.seconds() > InputValues.LIFT_FLYWHEEL_WAIT_MILLISECONDS) {
//                    turretScoring = Scoring.SEARCHING;
//                }

                break;
        }
    }

    public void shootArtifact() {
        leftLiftFlywheel.setPower(1.0);
        rightLiftFlywheel.setPower(1.0);
        Helper.sleep(InputValues.LIFT_FLYWHEEL_WAIT_MILLISECONDS);
        leftLiftFlywheel.setPower(0);
        rightLiftFlywheel.setPower(0);
    }
    public double flywheelVelocity(double distanceToGoal) {
        double[] D = {0, 12, 24, 36, 48, 60, 72, 84, 96, 108, 120, 132, 144, 156, 168, 180};
        double[] velocity = {1500, 1700, 1900, 2100, 2300, 2500, 2700, 2900, 3100, 3300, 3500,
                3700, 3900, 4100, 4300, 4500};
        int index = (int) Math.floor(distanceToGoal/12);

        if (index < 1) {
            return velocity[0];
        } else if (index > 15) {
            return velocity[15];
        }

        double flywheelV = (velocity[index + 1] - velocity[index]) / (D[index] - D[index-1]) *
                (distanceToGoal - D[index - 1]);

        return flywheelV;

    }
    public void resetTurretToZero() {
        turretMotor.setPower(0.8);
        turretMotor.setTargetPosition(0);
    }

    // Uses the position of the aprilTag to adjust the turret motor and center the aprilTag in the camera view

    public void adjustTurret() {

        // get heading, x pos, and y pos

        // --- A) Get the robot's current pose in field coordinates ---
        mecanumDrive.updatePoseEstimate();
        Pose2d robotPose = mecanumDrive.localizer.getPose();
        double robotX = robotPose.position.x;
        double robotY = robotPose.position.y;
        double robotHeadingRad = robotPose.heading.toDouble();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad); // 0 = +X, CCW+

        // --- B) Compute TURRET position in field coordinates ---
        //
        // turretOffsetXRobot, turretOffsetYRobot are in ROBOT frame:
        //  - +X is forward from robot center
        //  - +Y is left from robot center
        //
        // We rotate this offset by the robot's heading to convert it
        // into FIELD coordinates, then add it to the robot's position.
        double headingRad = Math.toRadians(robotHeadingDeg);
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);

        // Rotation of (x_r, y_r) into field frame:
        // x_f = x_r * cosθ - y_r * sinθ
        // y_f = x_r * sinθ + y_r * cosθ
        double turretOffsetXField = InputValues.TURRET_OFFSET_X * cosH - InputValues.TURRET_OFFSET_Y * sinH;
        double turretOffsetYField = InputValues.TURRET_OFFSET_X * sinH + InputValues.TURRET_OFFSET_Y * cosH;

        double turretX = robotX + turretOffsetXField;
        double turretY = robotY + turretOffsetYField;

        // --- C) Vector from TURRET to GOAL in field coordinates ---
        double dx = goalPosition.x - turretX;
        double dy = goalPosition.y - turretY;

        // Angle from turret to goal in field frame
        desiredFieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // --- D) Convert field angle to turret angle RELATIVE TO ROBOT ---
        double desiredTurretRelRobotDeg = desiredFieldAngleDeg - robotHeadingDeg;

        // --- E) Convert to angle from encoder zero ---
        double deltaFromZeroDeg = desiredTurretRelRobotDeg - turretZeroRelRobotDeg;

        // --- F) Angle → ticks and command motor ---
        int targetTicks = (int) Math.round(deltaFromZeroDeg * InputValues.TICKS_PER_DEGREE);

        telemetry.addData("desiredFieldAngleDeg", desiredFieldAngleDeg);
        telemetry.addData("robotHeadingDeg", robotHeadingDeg);
        telemetry.addData("desiredTurretRelRobotDeg", desiredTurretRelRobotDeg);
        telemetry.addData("deltaFromZeroDeg", deltaFromZeroDeg);
        telemetry.addData("targetTicks", targetTicks);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setPower(0.5); // tune as needed
    }

    public double getTurretDegrees() {
        return this.desiredFieldAngleDeg;
    }
}


