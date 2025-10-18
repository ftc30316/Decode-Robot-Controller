package org.firstinspires.ftc.teamcode.evergreendynamics;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Turret {

    //Setting up the state machines for the two states, aiming the turret towards the goal and shooting the artifacts to score
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

    public Aiming turretAiming = Turret.Aiming.AIMING;
    public Scoring turretScoring = Turret.Scoring.SEARCHING;
    private Telemetry telemetry;
    private DistanceSensor pistonSensor;
    private Servo servo;
    private String nextColorArtifact = "purple";
    public ElapsedTime pistonTimer = new ElapsedTime();

    public volatile Gamepad gamepad1 = null;
    AprilTagProcessor myAprilTagProcessor;
    DcMotorEx turretMotor;
    private DcMotorEx flywheel1;
    private Servo piston;
    public int aimAtTagId;
    public Thread backgroundThread;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, int aimAtTagId) {

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.aimAtTagId = aimAtTagId;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        //turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        piston = hardwareMap.get(Servo.class, "piston");
        pistonSensor = hardwareMap.get(DistanceSensor.class, "distancesensor1");

        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);



        // Create the AprilTag processor and assign it to a variable.
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessor = myAprilTagProcessorBuilder
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(getDecodeTagLibrary())
                .build();

        // Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
        VisionPortal myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Leafy")) // Logitech
                .setCameraResolution(new Size(800, 600)) // try 1080p for more pixels
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(myAprilTagProcessor)
                .setAutoStopLiveView(false)
                .build();


        // Enable or disable the AprilTag processor.
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

        // Reset the encoder during initialization
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set the amount of ticks to move
        turretMotor.setTargetPosition(0);

        // Switch to RUN_TO_POSITION mode
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to ___ ticks per second
        turretMotor.setPower(InputValues.TURRET_SPEED);

        // Creates a background thread so that while the robot is driving, intaking, and sorting, the turret can always be auto-locked on the goal
        this.backgroundThread = new Thread(this::constantlyAimAtAprilTag);

    }

    // Creates a loop that always aims at the goal
    public void constantlyAimAtAprilTag() {
        while (true) {
            aim(aimAtTagId);
        }
    }

    // Starts the flywheel
    public void startFlywheel() {
        flywheel1.setVelocity(InputValues.FLYWHEEL_SPEED);
    }

    // Pulls all of the possible aprilTag values
    public static AprilTagLibrary getDecodeTagLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(20, "BlueGoal",
                        6, DistanceUnit.INCH)
                .addTag(24, "RedGoal",
                        6, DistanceUnit.INCH)
                .addTag(21, "GPP",
                        6, DistanceUnit.INCH)
                .addTag(22, "PGP",
                        6, DistanceUnit.INCH)
                .addTag(23, "PPG",
                        6, DistanceUnit.INCH)
                .build();
    }

    // Uses the camera to look at the obelisk and determine if the motif pattern is GPP, PGP, or PPG - last resort is GPP (21)
    public int determineMotif() {
        List<AprilTagDetection> myAprilTagDetections;  // list of all detections
        AprilTagDetection myAprilTagDetection;         // current detection in for() loop
        int myAprilTagIdCode;                           // ID code of current detection, in for() loop

        for (int loopCounter = 0; loopCounter < 5; loopCounter++) {
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

    // A state machine that checks if there is an artifact in the piston slot, and moves artifact into flywheel when x is pressed
    public void score() {
        telemetry.addData("piston distance", pistonSensor.getDistance(DistanceUnit.INCH));
        switch (turretScoring) {
            case SEARCHING:
                if (pistonSensor.getDistance(DistanceUnit.INCH) < 5.5) {
                    turretScoring = Scoring.HOLDING;
                }
                break;
            case HOLDING:
                if (gamepad1.cross && pistonSensor.getDistance(DistanceUnit.INCH) < 5.5) {
                    turretScoring = Scoring.SHOOTING;
                }
                break;
            case SHOOTING:
                piston.setPosition(1);
                pistonTimer.reset();
                turretScoring = Scoring.RESETTING;
                break;
            case RESETTING:
                if (pistonTimer.seconds() > InputValues.PISTON_TRAVEL_TIME) {
                    piston.setPosition(0);
                    turretScoring = Scoring.SEARCHING;
                }
                break;
        }
    }

    // Switches the turret state to shooting, where the artifact will move into the flywheel
    public void shootArtifact() {
        if (pistonSensor.getDistance(DistanceUnit.INCH) < 5.5) {
            turretScoring = Scoring.SHOOTING;
        }
    }

    // Uses the position of the aprilTag to adjust the turret motor and center the aprilTag in the camera view
    public void adjustTurret (int aimAtTagId) {
        List<AprilTagDetection> myAprilTagDetections;  // list of all detections
        AprilTagDetection myAprilTagDetection;         // current detection in for() loop
        int myAprilTagIdCode;                           // ID code of current detection, in for() loop

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();


        telemetry.addLine(String.valueOf(myAprilTagDetections.size()));

        for (int i = 0; i < myAprilTagDetections.size(); i++) {
            myAprilTagDetection = myAprilTagDetections.get(i);
            double myTagPoseRange = myAprilTagDetection.ftcPose.range;
            double myTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
            double myTagPoseElevation = myAprilTagDetection.ftcPose.elevation;

            myAprilTagIdCode = myAprilTagDetection.id;

            if (myAprilTagIdCode == aimAtTagId) {
                double currentPosition = turretMotor.getCurrentPosition();
                double motorTicks = myAprilTagDetection.ftcPose.bearing * 5.3277777778;
                double newPosition = currentPosition - motorTicks;

                turretMotor.setTargetPosition((int) newPosition);

                telemetry.addLine(String.valueOf(myAprilTagIdCode));
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
                    Thread.sleep(25);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}

