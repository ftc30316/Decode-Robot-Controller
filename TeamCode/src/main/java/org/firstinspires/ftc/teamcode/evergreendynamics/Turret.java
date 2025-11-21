package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Turret {

    // Setting up the state machines for the two states, aiming the turret towards the goal and shooting the artifacts to score
    public enum Aiming {
        DECODING,
        AIMING
    }
    public enum Scoring {
        SEARCHING,
        HOLDING,
        SHOOTING,
        RESETTING
    }
    public enum FlywheelState {
        ON,
        OFF,
        REVERSE
    }
    public enum TurretLockingState {
        AUTO,
        MANUAL
    }
    FlywheelState flywheelState = FlywheelState.ON;
    TurretLockingState turretLockingState = TurretLockingState.AUTO;

    public Aiming turretAiming = Turret.Aiming.AIMING;
    public Scoring turretScoring = Turret.Scoring.SEARCHING;
    private Telemetry telemetry;
    private DistanceSensor liftSensor;
    public ElapsedTime liftTimer = new ElapsedTime();

    public volatile Gamepad gamepad1 = null;

    public volatile Gamepad gamepad2 = null;
    AprilTagProcessor myAprilTagProcessor;
    DcMotorEx turretMotor;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private MecanumDrive mecanumDrive;
    private Servo lift;
    public Vector2d goalPosition;
    public Thread turretBackgroundThread;
    private Limelight3A limelight;

    private volatile boolean runAutoAimThread = true;
    private volatile double turretDegrees = 0;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, Vector2d goalPosition, MecanumDrive mecanumDrive) {

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.goalPosition = goalPosition;
        this.mecanumDrive = mecanumDrive;
        this.limelight = limelight;

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        lift = hardwareMap.get(Servo.class, "lift");
        //liftSensor = hardwareMap.get(DistanceSensor.class, "distancesensor1");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        //Starts polling for data.

        limelight.start();

        // Reset the encoder during initialization
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the amount of ticks to move
        turretMotor.setTargetPosition(0);

        // Switch to RUN_TO_POSITION mode
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        // Start the motor moving by setting the max velocity to ___ ticks per second
//        turretMotor.setPower(InputValues.TURRET_MOTOR_POWER);

        lift.setPosition(InputValues.LIFT_START_POS);
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
                    try {
                        Thread.sleep((long) (InputValues.TURRET_THREAD_SLEEP_TIME));
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            } catch (Exception e) {

            } finally {
                turretMotor.setPower(0);
            }
        }
    }

    public void turretControl() {
        telemetry.addData("Turret locking state: ", turretLockingState);
        telemetry.addData("LEFT TRIGGER: ", gamepad2.left_bumper);
        telemetry.addData("RIGHT TRIGGER: ", gamepad2.right_bumper);
        switch (turretLockingState) {
            case AUTO:
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                adjustTurret();
                if (gamepad1.dpad_up) {
                    turretLockingState = TurretLockingState.MANUAL;
                }
                break;
            case MANUAL:
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

                if (gamepad2.dpad_up) {
                    turretLockingState = TurretLockingState.AUTO;
                }
                break;
        }
    }

    // Starts the flywheel
    public void startFlywheel() {
        leftFlywheel.setVelocity(InputValues.FLYWHEEL_SPEED);
        rightFlywheel.setVelocity(InputValues.FLYWHEEL_SPEED);
    }

    public void triggerFlywheel() {
        telemetry.addData("Flywheel state is: ", flywheelState);
        switch (flywheelState) {
//            case OFF:
//                leftFlywheel.setVelocity(0);
//                rightFlywheel.setVelocity(0);
//                if (gamepad1.x) {
//                    flywheelState = FlywheelState.ON;
//
//                }
//                break;
            case ON:
                leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

                leftFlywheel.setVelocity(InputValues.FLYWHEEL_SPEED);
                rightFlywheel.setVelocity(InputValues.FLYWHEEL_SPEED);
                if (gamepad2.right_trigger > 0.5) {
                    flywheelState = FlywheelState.REVERSE;
                }
                break;
            case REVERSE:
                leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

                leftFlywheel.setVelocity(InputValues.SLOW_FLYWHEEL_SPEED);
                rightFlywheel.setVelocity(InputValues.SLOW_FLYWHEEL_SPEED);
                if (gamepad2.left_trigger > 0.5) {
                    flywheelState = FlywheelState.ON;
                }
//                if (gamepad1.circleWasPressed()) {
//                    flywheelState = FlywheelState.OFF;
//                }
                break;
        }
    }

    // Uses the camera to look at the obelisk and determine if the motif pattern is GPP, PGP, or PPG - last resort is GPP (21)
    public int determineMotif() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    if (id == 21 || id == 22 || id == 23) {
                        telemetry.addData("Detected Motif", id);
                        return id;
                    }
                }
            }
        }

        telemetry.addData("LAST RESORT", 21);
        return 21;
    }

    // A state machine that auto-locks on goal
    public void aim(int aimAtTagId) {

        switch (turretAiming) {
            case DECODING:
                break;
            case AIMING:
                adjustTurret();
                break;
        }
    }

    // A state machine that checks if there is an artifact in the lift slot, and moves artifact into flywheel when x is pressed
    public void score() {
        telemetry.addData("score state", turretScoring);
        switch (turretScoring) {
            case SEARCHING:
                turretScoring = Scoring.HOLDING;
                break;
            case HOLDING:
                // && liftSensor.getDistance(DistanceUnit.INCH) < 5.5
                if (gamepad1.cross) {
                    turretScoring = Scoring.SHOOTING;
                }
                break;
            case SHOOTING:
                lift.setPosition(InputValues.LIFT_END_POS);
                liftTimer.reset();
                turretScoring = Scoring.RESETTING;
                break;
            case RESETTING:
                if (liftTimer.seconds() > InputValues.LIFT_TRAVEL_TIME) {
                    lift.setPosition(0);
                    turretScoring = Scoring.SEARCHING;
                }
                break;
        }
    }

    // Switches the turret state to shooting, where the artifact will move into the flywheel
    public void switchTurretStateShooting() {
            turretScoring = Scoring.SHOOTING;
            score();
    }

    public void shootArtifact() {
        switchTurretStateShooting();
        try {
            Thread.sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        score(); // resets lift servo
        try {
            Thread.sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    // Uses the position of the aprilTag to adjust the turret motor and center the aprilTag in the camera view
    public void adjustTurret() {
        mecanumDrive.updatePoseEstimate();
        turretMotor.setPower(InputValues.TURRET_MOTOR_POWER);
        // get heading, x pos, and y pos
        Pose2d robotPose = mecanumDrive.localizer.getPose();
        double robotX = robotPose.position.x;
        double robotY = robotPose.position.y;
        double robotHeadingRad = robotPose.heading.toDouble();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        // get goal position; blue: -66, -66, red: -66, 66
        double goalX = goalPosition.x;
        double goalY = goalPosition.y;
        double turretStartingDegrees = -90;

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
        double aimingDegrees = turretStartingDegrees - theta - robotHeadingDeg;
        turretDegrees = aimingDegrees;

        // turret starting heading: 0, heading is always relative to robot
        // move turret to new heading
        turretMotor.setTargetPosition((int) ((aimingDegrees) * InputValues.TICKS_PER_DEGREE));

        // alter flywheel velocity based on distance computed
        double flywheelAdjustingSpeed = InputValues.FLYWHEEL_SLOPE * D + InputValues.FLYWHEEL_Y_INTERCEPT;
        leftFlywheel.setVelocity(flywheelAdjustingSpeed);
        rightFlywheel.setVelocity(flywheelAdjustingSpeed);

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

