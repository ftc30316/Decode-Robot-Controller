package org.firstinspires.ftc.teamcode.evergreendynamics;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

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
    private Servo lift;
    public int aimAtTagId;
    public Thread backgroundThread;
    VisionPortal myVisionPortal;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int aimAtTagId) {

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.aimAtTagId = aimAtTagId;

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        //turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        lift = hardwareMap.get(Servo.class, "lift");
        //liftSensor = hardwareMap.get(DistanceSensor.class, "distancesensor1");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Create the AprilTag processor and assign it to a variable.
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessor = myAprilTagProcessorBuilder
                .setDrawTagID(false)
                .setDrawTagOutline(false)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        myAprilTagProcessor.setDecimation(2); // Set dynamically later

        // Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Leafy")) // Logitech
                .setCameraResolution(new Size(800, 600)) // try 1080p for more pixels
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(myAprilTagProcessor)
                .setAutoStopLiveView(false)

                .build();

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

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
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

        // Reset the encoder during initialization
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the amount of ticks to move
        turretMotor.setTargetPosition(0);

        // Switch to RUN_TO_POSITION mode
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to ___ ticks per second
        turretMotor.setPower(InputValues.TURRET_SPEED);

        lift.setPosition(InputValues.LIFT_START_POS);

        // Creates a background thread so that while the robot is driving, intaking, and sorting, the turret can always be auto-locked on the goal
        this.backgroundThread = new Thread(this::constantlyAimAtAprilTag);

    }

    int framesSinceSeen = 0;
    int decimation = 2;
    long fpsStart = System.nanoTime();
    int  fpsFrames = 0; double fps = 0;

    void sendFpsToDashboard(FtcDashboard dash) {
        ++fpsFrames;
        if (System.nanoTime() - fpsStart >= 1_000_000_000L) {
            fps = fpsFrames / ((System.nanoTime() - fpsStart) / 1e9);
            fpsFrames = 0; fpsStart = System.nanoTime();
        }
        TelemetryPacket p = new TelemetryPacket();
        p.put("Vision FPS", fps);
        dash.sendTelemetryPacket(p);
    }

    void updateDecimation(List<AprilTagDetection> dets) {
        if (dets.isEmpty()) {
            if (++framesSinceSeen > 15 && decimation != 1) {
                decimation = 1;
                myAprilTagProcessor.setDecimation(1);
            }
            return;
        }
        framesSinceSeen = 0;
        double rangeIn = dets.get(0).ftcPose != null ? dets.get(0).ftcPose.range : 9999;
        int newDec = (rangeIn < 36) ? 3 : 2;
        if (newDec != decimation) {
            decimation = newDec;
            myAprilTagProcessor.setDecimation(newDec);
        }
    }

    // Creates a loop that always aims at the goal
    public void constantlyAimAtAprilTag() {
        while (true) {
            aim(aimAtTagId);
        }
    }

    public void turretControl() {
        telemetry.addData("Turret locking state: ", turretLockingState);
        telemetry.addData("LEFT TRIGGER: ", gamepad2.left_bumper);
        telemetry.addData("RIGHT TRIGGER: ", gamepad2.right_bumper);
        switch (turretLockingState) {
            case AUTO:
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                adjustTurret(aimAtTagId);
                if (gamepad1.triangle) {
                    turretLockingState = TurretLockingState.MANUAL;
                }
                break;
            case MANUAL:
                turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (gamepad2.left_bumper) {
                    turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    turretMotor.setPower(InputValues.TURRET_SPEED);
                }
                else if (gamepad2.right_bumper) {
                    turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    turretMotor.setPower(InputValues.TURRET_SPEED);
                }
                else if (gamepad2.left_stick_x != 0) {
                    double turretJoystickPower = gamepad2.left_stick_x;
                    turretMotor.setPower(turretJoystickPower);
                }
                else {
                    turretMotor.setPower(0);
                }

                if (gamepad2.triangle) {
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
            case OFF:
                leftFlywheel.setVelocity(0);
                rightFlywheel.setVelocity(0);
                if (gamepad1.circleWasPressed()) {
                    flywheelState = FlywheelState.ON;
                }
                break;
            case ON:
                leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
                rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

                leftFlywheel.setVelocity(InputValues.FLYWHEEL_SPEED);
                rightFlywheel.setVelocity(InputValues.FLYWHEEL_SPEED);
                if (gamepad1.circleWasPressed()) {
                    flywheelState = FlywheelState.REVERSE;
                }
                break;
            case REVERSE:
                leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

                leftFlywheel.setVelocity(InputValues.SLOW_FLYWHEEL_SPEED);
                rightFlywheel.setVelocity(InputValues.SLOW_FLYWHEEL_SPEED);
                if (gamepad1.circleWasPressed()) {
                    flywheelState = FlywheelState.OFF;
                }
                break;
        }
    }

    // Uses the camera to look at the obelisk and determine if the motif pattern is GPP, PGP, or PPG - last resort is GPP (21)
    public int determineMotif() {
        List<AprilTagDetection> myAprilTagDetections;  // list of all detections
        AprilTagDetection myAprilTagDetection;         // current detection in for() loop
        int myAprilTagIdCode;                           // ID code of current detection, in for() loop

        for (int loopCounter = 0; loopCounter < 3; loopCounter++) {
            // Get a list of AprilTag detections.
            myAprilTagDetections = myAprilTagProcessor.getDetections();

            telemetry.addLine(String.valueOf(myAprilTagDetections.size()));

            for (int i = 0; i < myAprilTagDetections.size(); i++) {
                myAprilTagDetection = myAprilTagDetections.get(i);

                if ((myAprilTagDetection.id == 21) || (myAprilTagDetection.id == 22) || (myAprilTagDetection.id == 23)) {
//                if (myAprilTagDetection.id == 20) {
                    return (myAprilTagDetection.id);
                }

            }
            try {
                Thread.sleep(InputValues.MOTIF_LOOP_WAIT);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
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
                adjustTurret(aimAtTagId);
                break;
        }
    }

    // A state machine that checks if there is an artifact in the lift slot, and moves artifact into flywheel when x is pressed
    public void score() {
        //telemetry.addData("lift distance", liftSensor.getDistance(DistanceUnit.INCH));
        switch (turretScoring) {
            case SEARCHING:
                //if (liftSensor.getDistance(DistanceUnit.INCH) < 5.5) {
                    turretScoring = Scoring.HOLDING;
                //}
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
    public void shootArtifact() {
            turretScoring = Scoring.SHOOTING;
            score();
    }

    // Uses the position of the aprilTag to adjust the turret motor and center the aprilTag in the camera view
    public void adjustTurret (int aimAtTagId) {
        List<AprilTagDetection> myAprilTagDetections;  // list of all detections
        AprilTagDetection myAprilTagDetection;         // current detection in for() loop
        int myAprilTagIdCode;                           // ID code of current detection, in for() loop

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();

        double fps = myVisionPortal.getFps();
        TelemetryPacket cameraData = new TelemetryPacket();
        cameraData.put("Vision FPS", fps);
        FtcDashboard.getInstance().sendTelemetryPacket(cameraData);

        telemetry.addData("Vision FPS", fps);

        for (int i = 0; i < myAprilTagDetections.size(); i++) {
            myAprilTagDetection = myAprilTagDetections.get(i);

            myAprilTagIdCode = myAprilTagDetection.id;

            TelemetryPacket aprilTagData = new TelemetryPacket();
            aprilTagData.put("number of tags seen", String.valueOf(myAprilTagDetections.size()));
            aprilTagData.put("which tags seen", String.valueOf(myAprilTagIdCode));
            FtcDashboard.getInstance().sendTelemetryPacket(aprilTagData);

            if (myAprilTagIdCode == aimAtTagId) {
                double currentPosition = turretMotor.getCurrentPosition();
                double motorTicks = myAprilTagDetection.ftcPose.bearing * 5.3277777778;
                double newPosition = currentPosition - motorTicks;

                turretMotor.setTargetPosition((int) newPosition);

                telemetry.addLine(String.valueOf(myAprilTagIdCode));
                telemetry.addData("bearing", myAprilTagDetection.ftcPose.bearing); // angle the camera must turn (left/right) to face target)

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("range", myAprilTagDetection.ftcPose.range); // distance from camera to target
                packet.put("bearing", myAprilTagDetection.ftcPose.bearing); // angle the camera must turn (left/right) to face target
                packet.put("elevation", myAprilTagDetection.ftcPose.elevation); // angle the camera must tilt (up/down) to face target
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                packet = new TelemetryPacket();
                packet.put("velocity", turretMotor.getVelocity());
                packet.put("position", turretMotor.getCurrentPosition());
                packet.put("is at target", !turretMotor.isBusy());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                try {
                    Thread.sleep(InputValues.SLEEP_PER_AUTO_FRAMES);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}

