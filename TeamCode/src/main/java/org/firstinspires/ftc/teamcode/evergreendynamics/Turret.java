package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    VisionPortal myVisionPortal;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, Vector2d goalPosition, MecanumDrive mecanumDrive) {

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.goalPosition = goalPosition;
        this.mecanumDrive = mecanumDrive;

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


        // Create the AprilTag processor and assign it to a variable.
//        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//        myAprilTagProcessor = myAprilTagProcessorBuilder
//                .setDrawTagID(false)
//                .setDrawTagOutline(false)
//                .setDrawAxes(false)
//                .setDrawCubeProjection(false)
//                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .build();
//
//        myAprilTagProcessor.setDecimation(2); // Set dynamically later
//
//        // Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
//        myVisionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Leafy")) // Logitech
//                .setCameraResolution(new Size(800, 600)) // try 1080p for more pixels
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .addProcessor(myAprilTagProcessor)
//                .setAutoStopLiveView(false)
//
//                .build();
//
//        try {
//            Thread.sleep(2000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }

//        ExposureControl exp = myVisionPortal.getCameraControl(ExposureControl.class);
//        if (exp != null && exp.getMode() != ExposureControl.Mode.Manual) {
//            exp.setMode(ExposureControl.Mode.Manual);
//            exp.setExposure(InputValues.EXPOSURE, TimeUnit.MILLISECONDS);
//        }
//
//        GainControl gain = myVisionPortal.getCameraControl(GainControl.class);
//        if (gain != null) gain.setGain(InputValues.GAIN);
//
//        WhiteBalanceControl wb = myVisionPortal.getCameraControl(WhiteBalanceControl.class);
//        if (wb != null) {
//            wb.setMode(WhiteBalanceControl.Mode.MANUAL);
//            wb.setWhiteBalanceTemperature(InputValues.WHITE_BALANCE);
//        }
//
//        CameraControl cameraControl = myVisionPortal.getCameraControl(CameraControl.class);
//
//        FocusControl focusControl = myVisionPortal.getCameraControl(FocusControl.class);
//        if (focusControl != null) {
//            focusControl.setMode(FocusControl.Mode.Infinity);
//            focusControl.setFocusLength(InputValues.FOCUS);
//        }
//
//        telemetry.addData("Exposure(ms)", exp != null ? exp.getExposure(TimeUnit.MILLISECONDS) : -1);
//        telemetry.addData("Gain", gain != null ? gain.getGain() : -1);
//        telemetry.addData("WB(K)", wb != null ? wb.getWhiteBalanceTemperature() : -1);

        // Enable or disable the AprilTag processor.
        //myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

        // Reset the encoder during initialization
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the amount of ticks to move
        turretMotor.setTargetPosition(0);

        // Switch to RUN_TO_POSITION mode
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to ___ ticks per second
        turretMotor.setPower(InputValues.FLYWHEEL_SLOPE);

        lift.setPosition(InputValues.LIFT_START_POS);

        // Creates a background thread so that while the robot is driving, intaking, and sorting, the turret can always be auto-locked on the goal
        this.turretBackgroundThread = new Thread(this::constantlyAdjustTurret);
    }

    public void constantlyAdjustTurret() {
        while (true) {
            adjustTurret();
        }
    }

//    int framesSinceSeen = 0;
//    int decimation = 2;
//    long fpsStart = System.nanoTime();
//    int  fpsFrames = 0; double fps = 0;

    void sendFpsToDashboard(FtcDashboard dash) {
//        ++fpsFrames;
//        if (System.nanoTime() - fpsStart >= 1_000_000_000L) {
//            fps = fpsFrames / ((System.nanoTime() - fpsStart) / 1e9);
//            fpsFrames = 0; fpsStart = System.nanoTime();
//        }
//        TelemetryPacket p = new TelemetryPacket();
//        p.put("Vision FPS", fps);
//        dash.sendTelemetryPacket(p);
    }

    void updateDecimation(List<AprilTagDetection> dets) {
//        if (dets.isEmpty()) {
//            if (++framesSinceSeen > 15 && decimation != 1) {
//                decimation = 1;
//                myAprilTagProcessor.setDecimation(1);
//            }
//            return;
//        }
//        framesSinceSeen = 0;
//        double rangeIn = dets.get(0).ftcPose != null ? dets.get(0).ftcPose.range : 9999;
//        int newDec = (rangeIn < 36) ? 3 : 2;
//        if (newDec != decimation) {
//            decimation = newDec;
//            myAprilTagProcessor.setDecimation(newDec);
//        }
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
//        List<AprilTagDetection> myAprilTagDetections;  // list of all detections
//        AprilTagDetection myAprilTagDetection;         // current detection in for() loop
//        int myAprilTagIdCode;                           // ID code of current detection, in for() loop
//
//        for (int loopCounter = 0; loopCounter < 3; loopCounter++) {
//            // Get a list of AprilTag detections.
//            myAprilTagDetections = myAprilTagProcessor.getDetections();
//
//            telemetry.addLine(String.valueOf(myAprilTagDetections.size()));
//
//            for (int i = 0; i < myAprilTagDetections.size(); i++) {
//                myAprilTagDetection = myAprilTagDetections.get(i);
//
//                if ((myAprilTagDetection.id == 21) || (myAprilTagDetection.id == 22) || (myAprilTagDetection.id == 23)) {
////                if (myAprilTagDetection.id == 20) {
//                    return (myAprilTagDetection.id);
//                }
//
//            }
//            try {
//                Thread.sleep(InputValues.MOTIF_LOOP_WAIT);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        }

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
            sleep((long) (InputValues.LIFT_TRAVEL_TIME * 1000));
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
        double robotHeading = Math.toDegrees(robotPose.heading.toDouble());

        // get goal position; blue: -66, -66, red: -66, 66
        double goalX = goalPosition.x;
        double goalY = goalPosition.y;
        double turretStartingDegrees = -90;

        // find A: (Yr - Yb)
        double A = robotY - goalY;

        // find C: (Xr - Xb)
        double C = robotX - goalX;

        // find distance (D): sqrt(A^2 + C^2)
        double D = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        // solve for angle: c = acos(A/D); in field coordinates
        double theta = Math.toDegrees(Math.acos(A/D));

        // aiming degrees = -90 - theta; in field coordinates
        double aimingDegrees = turretStartingDegrees - theta - robotHeading;

        // turret starting heading: 0, heading is always relative to robot
        // move turret to new heading
        turretMotor.setTargetPosition((int) ((aimingDegrees) * InputValues.TICKS_PER_DEGREE));

        // alter flywheel velocity based on distance computed
        double flywheelAdjustingSpeed = InputValues.FLYWHEEL_SLOPE * D + InputValues.FLYWHEEL_Y_INTERCEPT;
        leftFlywheel.setVelocity(flywheelAdjustingSpeed);
        rightFlywheel.setVelocity(flywheelAdjustingSpeed);

        // telemetry
        TelemetryPacket turretValues = new TelemetryPacket();
        turretValues.put("robot heading", robotHeading);
        turretValues.put("robot X", robotX);
        turretValues.put("robot Y", robotY);

        turretValues.put("goal X", goalX);
        turretValues.put("goal Y", goalY);

        turretValues.put("A", A);
        turretValues.put("C", C);
        turretValues.put("Distance", D);
        turretValues.put("angle calculated", theta);

        turretValues.put("aiming degrees", aimingDegrees);
        FtcDashboard.getInstance().sendTelemetryPacket(turretValues);

    }
}

