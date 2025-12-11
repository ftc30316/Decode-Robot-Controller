package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import android.renderscript.ScriptGroup;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Turret {
    // Setting up the state machines for the two states, aiming the turret towards the goal and shooting the artifacts to score

    public enum FlywheelState {
        ON,

        OFF
    }
    public enum LiftWheelState {
        ON,

        OFF
    }
    public enum TurretLockingState {
        AUTO,
        MANUAL
    }
    FlywheelState flywheelState = FlywheelState.ON;
    LiftWheelState liftWheelState = LiftWheelState.OFF;
    TurretLockingState turretLockingState = TurretLockingState.AUTO;
    private Telemetry telemetry;

    public Intake intake;

    public volatile Gamepad gamepad1 = null;

    public volatile Gamepad gamepad2 = null;
    DcMotorEx turretMotor;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private CRServo leftLiftWheel;
    private CRServo rightLiftWheel;
    public int artifactsWhenCrossWasPressed = 0;
    public int artifactsWhenShooting = 0;
    private MecanumDrive mecanumDrive;
    public Vector2d goalPosition;

    public Thread turretBackgroundThread;
    private volatile boolean runAutoAimThread = true;
    private volatile double turretDegrees = 0;
    private double turretZeroRelRobotDeg;
    double desiredFieldAngleDeg;

    public ElapsedTime liftWheelTimer = new ElapsedTime();

    public Turret(HardwareMap hardwareMap, Telemetry telemetry,
                  Gamepad gamepad1, Gamepad gamepad2,
                  Vector2d goalPosition,
                  MecanumDrive mecanumDrive, Intake intake) {

        this.telemetry = telemetry;
        this.intake = intake;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.goalPosition = goalPosition;
        this.mecanumDrive = mecanumDrive;

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        leftLiftWheel = hardwareMap.get(CRServo.class, "leftLiftServo");
        rightLiftWheel = hardwareMap.get(CRServo.class, "rightLiftServo");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLiftWheel.setDirection(CRServo.Direction.FORWARD);
        rightLiftWheel.setDirection(CRServo.Direction.REVERSE);

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

    // Starts the flywheel
    public void loop() {
        telemetry.addData("Flywheel: ", flywheelState);
        telemetry.addData("Lift: ", liftWheelState);
        telemetry.addData("Turret: ", turretLockingState);
        // State machine for the FLY wheels
        switch (flywheelState) {
            case ON:
                if (gamepad1.circleWasPressed()) {
                    flywheelState = FlywheelState.OFF;
                }
                break;
            case OFF:
                leftFlywheel.setVelocity(0);
                rightFlywheel.setVelocity(0);
                if (gamepad1.circleWasPressed()) {
                    flywheelState = FlywheelState.ON;
                }
        }
        // State machine for the LIFT wheels
        switch (liftWheelState) {
            case ON:
                leftLiftWheel.setPower(1.0);
                rightLiftWheel.setPower(1.0);
                if (liftWheelTimer.seconds() > InputValues.LIFT_WHEEL_WAIT_SECONDS * artifactsWhenCrossWasPressed) {
                    liftWheelState = LiftWheelState.OFF;
                }
                break;
            case OFF:
                leftLiftWheel.setPower(0);
                rightLiftWheel.setPower(0);
                if (gamepad1.crossWasPressed()) {
                    artifactsWhenCrossWasPressed = intake.getNumberOfArtifacts();
                    liftWheelTimer.reset();
                    liftWheelState = LiftWheelState.ON;
                }
        }
        // State machine for turret locking state: Auto or Manual
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

    public void shoot() {
        artifactsWhenShooting = intake.getNumberOfArtifacts();
        telemetry.addData("Artifacts when shooting: ", artifactsWhenShooting);
        leftLiftWheel.setPower(1.0);
        rightLiftWheel.setPower(1.0);
        Helper.sleep(InputValues.LIFT_WHEEL_WAIT_MILLISECONDS * artifactsWhenShooting);
        leftLiftWheel.setPower(0);
        rightLiftWheel.setPower(0);
    }

    public double getFlywheelVelocity(double distanceToGoal) {
        double[] D = {          0,   12,   24,   36,   48,   60,   72,   84,   96,  108,  120,  132,  144,  156,  168,  180};
        double[] velocity = {1425, 1425, 1425, 1425, 1425, 1425, 1408, 1391, 1375, 1525, 1600, 1600, 1620, 1640, 1680, 1700};
        int index = (int) Math.floor(distanceToGoal/12);


        // Clamp to valid range for interpolation
        if (index <= 0) {
            index = 0;
        } else if (index >= D.length - 1) {
            // If we're at or beyond the last point, just return the last velocity
            return velocity[velocity.length - 1];
        }

        double x0 = D[index];
        double x1 = D[index + 1];
        double v0 = velocity[index];
        double v1 = velocity[index + 1];

        // Fraction between the two distance points
        double t = (distanceToGoal - x0) / (x1 - x0);  // should be between 0 and 1

        // Proper linear interpolation: v = v0 + t * (v1 - v0)
        double flywheelV = v0 + t * (v1 - v0);

        telemetry.addData("flywheel velocity", flywheelV);
        telemetry.addData("velocity index", v0);
        telemetry.addData("distance index", x0);
        telemetry.addData("index", index);

        if (InputValues.FLYWHEEL_TEST_ON) {
            if (gamepad1.dpadUpWasPressed()) {
                InputValues.FLYWHEEL_TEST_VELOCITY += 25;
            }
            if (gamepad1.dpadDownWasPressed()) {
                InputValues.FLYWHEEL_TEST_VELOCITY -= 25;
            }
            return InputValues.FLYWHEEL_TEST_VELOCITY;
        }
        else {
            return flywheelV;
        }

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

        double distanceFromGoal = Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2));
        double flywheelVelocity = getFlywheelVelocity(distanceFromGoal);

//        telemetry.addData("Goal distance: ", distanceFromGoal);
//        telemetry.addData("Flywheel velocity: ", flywheelVelocity);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setPower(0.5); // tune as needed

        leftFlywheel.setVelocity(flywheelVelocity);
        rightFlywheel.setVelocity(flywheelVelocity);

        TelemetryPacket turretDistanceAndVelocity = new TelemetryPacket();
        turretDistanceAndVelocity.put("Goal distance: ", distanceFromGoal);
        turretDistanceAndVelocity.put("Flywheel velocity", flywheelVelocity);
//        turretDistanceAndVelocity.put("desiredFieldAngleDeg", desiredFieldAngleDeg);
//        turretDistanceAndVelocity.put("robotHeadingDeg", robotHeadingDeg);
//        turretDistanceAndVelocity.put("desiredTurretRelRobotDeg", desiredTurretRelRobotDeg);
//        turretDistanceAndVelocity.put("deltaFromZeroDeg", deltaFromZeroDeg);
//        turretDistanceAndVelocity.put("targetTicks", targetTicks);
        FtcDashboard.getInstance().sendTelemetryPacket(turretDistanceAndVelocity);
    }

    public double getTurretDegrees() {
        return this.desiredFieldAngleDeg;
    }
}


