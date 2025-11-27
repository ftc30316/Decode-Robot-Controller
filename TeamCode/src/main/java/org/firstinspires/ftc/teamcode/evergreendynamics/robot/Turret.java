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
    private Servo lift;
    public Vector2d goalPosition;

    public Thread turretBackgroundThread;
    public int flywheelSpeed = 2000;
    private volatile boolean runAutoAimThread = true;
    private volatile double turretDegrees = 0;
    public ElapsedTime liftTimer = new ElapsedTime();

    double turretStartHeading;
    public Turret(HardwareMap hardwareMap, Telemetry telemetry,
                  Gamepad gamepad1, Gamepad gamepad2,
                  Vector2d goalPosition, double turretStartHeading,
                  MecanumDrive mecanumDrive) {

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.goalPosition = goalPosition;
        this.mecanumDrive = mecanumDrive;
        this.turretStartHeading = turretStartHeading;

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShootingFlywheel = hardwareMap.get(DcMotorEx.class, "leftShootingFlywheel");
        rightShootingFlywheel = hardwareMap.get(DcMotorEx.class, "rightShootingFlywheel");
        leftLiftFlywheel = hardwareMap.get(CRServo.class, "leftLiftFlywheel");
        rightLiftFlywheel = hardwareMap.get(CRServo.class, "leftLiftFlywheel");
        lift = hardwareMap.get(Servo.class, "lift");
        //liftSensor = hardwareMap.get(DistanceSensor.class, "distancesensor1");

        leftShootingFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShootingFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        leftShootingFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShootingFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLiftFlywheel.setDirection(CRServo.Direction.REVERSE);
        rightLiftFlywheel.setDirection(CRServo.Direction.FORWARD);

        // Reset the encoder during initialization
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the amount of ticks to move
        turretMotor.setTargetPosition(0);

        // Switch to RUN_TO_POSITION mode
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                leftShootingFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                rightShootingFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                leftShootingFlywheel.setVelocity(0);
                rightShootingFlywheel.setVelocity(0);
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
                leftLiftFlywheel.setPower(0);
                rightLiftFlywheel.setPower(0);
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
                leftLiftFlywheel.setPower(1.0);
                rightLiftFlywheel.setPower(1.0);
                if (liftTimer.seconds() > InputValues.LIFT_FLYWHEEL_WAIT_MILLISECONDS) {
                    turretScoring = Scoring.SEARCHING;
                }

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

    public void resetTurretToZero() {
        turretMotor.setPower(0.8);
        turretMotor.setTargetPosition(0);
    }

    // Uses the position of the aprilTag to adjust the turret motor and center the aprilTag in the camera view
    public void adjustTurret() {
        turretMotor.setPower(InputValues.TURRET_MOTOR_POWER);
        // get heading, x pos, and y pos
        mecanumDrive.updatePoseEstimate();
        Pose2d robotPose = mecanumDrive.localizer.getPose();
        double robotX = robotPose.position.x;
        double robotY = robotPose.position.y;
        double robotHeadingRad = robotPose.heading.toDouble();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        // get goal position; blue: -66, -66, red: -66, 66
        double goalX = goalPosition.x;
        double goalY = goalPosition.y;

        // compute turret field position using robot heading + Y offset
        double turretX = robotX - InputValues.TURRET_OFFSET_Y * Math.sin(robotHeadingRad);
        double turretY = robotY + InputValues.TURRET_OFFSET_Y * Math.cos(robotHeadingRad);

        // find A: (Yr - Yb)
        double A = robotY - goalY;

        // find C: (Xr - Xb)
        double C = robotX - goalX;

        // find distance (D): sqrt(A^2 + C^2)
        double D = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        double theta = Math.toDegrees(Math.acos(A/D));
        //double angleToGoal = Math.toDegrees(Math.atan2(A, C));

        // compute turret aiming angle relative to robot
        //double aimingDegrees = angleToGoal - robotHeadingDeg - turretStartingDegrees;
        double aimingDegrees = turretStartHeading - theta - robotHeadingDeg;
        turretDegrees = aimingDegrees + robotHeadingDeg; // added robotHeadingDeg

        // turret starting heading: 0, heading is always relative to robot
        // move turret to new heading
        turretMotor.setTargetPosition((int) ((aimingDegrees) * InputValues.TICKS_PER_DEGREE));

        // alter flywheel velocity based on distance computed
        double flywheelAdjustingSpeed = InputValues.FLYWHEEL_SLOPE * D + InputValues.FLYWHEEL_Y_INTERCEPT;
        leftShootingFlywheel.setVelocity(flywheelAdjustingSpeed);
        rightShootingFlywheel.setVelocity(flywheelAdjustingSpeed);

        // telemetry
        TelemetryPacket turretValues = new TelemetryPacket();
        turretValues.put("robot heading", robotHeadingDeg);
        turretValues.put("robot X", robotX);
        turretValues.put("robot Y", robotY);

        turretValues.put("turret X", turretX);
        turretValues.put("turret Y", turretY);

        turretValues.put("goal X", goalX);
        turretValues.put("goal Y", goalY);

        turretValues.put("A", A);
        turretValues.put("C", C);
        turretValues.put("Distance", D);
        turretValues.put("angle calculated", theta);

        turretValues.put("aiming degrees", aimingDegrees);
        FtcDashboard.getInstance().sendTelemetryPacket(turretValues);

    }

    public double getTurretDegrees() {
        return this.turretDegrees;
    }
}

