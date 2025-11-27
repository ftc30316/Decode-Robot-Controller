package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

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
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class TurretMathUpgrade {

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

    public Aiming turretAiming = TurretMathUpgrade.Aiming.AIMING;
    public Scoring turretScoring = TurretMathUpgrade.Scoring.SEARCHING;
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

    double turretStartHeading;



    // Turret angle **relative to robot** when encoder = 0 (computed at init)
    // K = turretFieldAngleAtStart - robotHeadingAtStart
    private double turretZeroRelRobotDeg;



    public TurretMathUpgrade(HardwareMap hardwareMap, Telemetry telemetry,
                             Gamepad gamepad1, Gamepad gamepad2,
                             Vector2d goalPosition, double turretFieldAngleStartDeg,
                             MecanumDrive mecanumDrive) {

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.goalPosition = goalPosition;
        this.mecanumDrive = mecanumDrive;
        this.limelight = limelight;
        this.turretStartHeading = turretStartHeading;

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

//        // Start the motor moving by setting the max velocity to ___ ticks per second
//        turretMotor.setPower(InputValues.TURRET_MOTOR_POWER);

        lift.setPosition(InputValues.LIFT_START_POS);
    }


    /**
     * Call once at the beginning of the match.
     *
     * @param robotHeadingStartDeg   robot's starting heading in field coords
     * @param turretFieldAngleStartDeg turret's starting aim angle in field coords
     *                                 (the direction the turret is pointing at init)
     */
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


    /**
     * Call this every loop (in your OpMode's loop() or robot periodic).
     * It updates the turret target so it keeps aiming at the goal.
     */
    public void update() {

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
        double desiredFieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

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

    // Normalize angle to (-180, 180] so the turret takes the shortest path
    private static double wrapDegrees(double angleDeg) {
        double a = angleDeg;
        while (a <= -180.0) a += 360.0;
        while (a > 180.0) a -= 360.0;
        return a;
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
                    //adjustTurret();
                    update();
                    try {
                        Thread.sleep((long) (InputValues.TURRET_THREAD_SLEEP_TIME_MILLIS));
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
                //turretBackgroundThread.start();
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                adjustTurret();
                if (gamepad1.dpad_up) {
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

//                if (gamepad2.dpad_up) {
//                    turretLockingState = TurretLockingState.AUTO;
//                }
                break;
        }
    }

    // Starts the flywheel
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
                if (gamepad2.right_trigger > 0.5) {
                    flywheelState = FlywheelState.REVERSE;
                }
                break;
            case REVERSE:
                leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
                rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
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

//        if (distanceToGoal > 0 && distanceToGoal <= 12) {
//            return (2000.0 - 1500.0) / (12.0 - 0.0) * (distanceToGoal);
//
//        } else if (distanceToGoal > 12 && distanceToGoal <= 24) {
//            return (2500.0 - 2000.0) / (24.0 - 12.0) * (distanceToGoal - 12);
//
//        } else if (distanceToGoal > 24 && distanceToGoal <= 36) {
//            return (3000.0 - 2500.0) / (36.0 - 24.0) * (distanceToGoal - 24);
//
//        } else if (distanceToGoal > 36 && distanceToGoal <= 48) {
//            return (3500.0 - 3000.0) / (48.0 - 36.0) * (distanceToGoal - 36);
//
//        } else if (distanceToGoal > 48 && distanceToGoal <= 60) {
//            return (4000.0 - 3500.0) / (60.0 - 48.0) * (distanceToGoal - 48);
//
//        } else if (distanceToGoal > 60 && distanceToGoal <= 72) {
//            return (4500.0 - 4000.0) / (72.0 - 60.0) * (distanceToGoal - 60);
//
//        } else if (distanceToGoal > 72 && distanceToGoal <= 84) {
//            return (5000.0 - 4500.0) / (84.0 - 72.0) * (distanceToGoal - 72);
//
//        } else if (distanceToGoal > 84 && distanceToGoal <= 96) {
//            return (5500.0 - 5000.0) / (96.0 - 84.0) * (distanceToGoal - 84);
//
//        } else if (distanceToGoal > 96 && distanceToGoal <= 108) {
//            return (5500.0 - 5000.0) / (96.0 - 84.0) * (distanceToGoal - 84);
//
//        }
    }

    public void resetTurretToZero() {
        turretMotor.setPower(0.8);
        turretMotor.setTargetPosition(0);
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
        double flywheelAdjustingSpeed = flywheelVelocity(D);
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

