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
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
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

    public enum TurretVelocityMode {
        AUTO,
        MANUAL
    }

    FlywheelState flywheelState = FlywheelState.ON;
    LiftWheelState liftWheelState = LiftWheelState.OFF;
    TurretLockingState turretLockingState = TurretLockingState.AUTO;
    private Telemetry telemetry;

    public Intake intake;
    public Keybinds keybinds;
    DcMotorEx turretMotor;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private CRServo leftLiftWheel;
    private CRServo rightLiftWheel;
    public int artifactsWhenCrossWasPressed = 0;
    public int artifactsWhenShooting = 0;
    public int manualVelocity = InputValues.STARTING_VELOCITY;
    private MecanumDrive mecanumDrive;
    public Vector2d goalPosition;
    private volatile double turretDegrees = 0;
    private double turretZeroRelRobotDeg;
    double desiredFieldAngleDeg;
    private Servo launchZoneLED;
    Datalog datalog = null;

    public double turretX = 0;
    public double turretY = 0;


    TurretVelocityMode turretVelocityMode = TurretVelocityMode.AUTO;
    public ElapsedTime liftWheelTimer = new ElapsedTime();

    public Turret(HardwareMap hardwareMap, Telemetry telemetry,
                  Keybinds keybinds, Vector2d goalPosition,
                  MecanumDrive mecanumDrive, Intake intake,
                  TurretVelocityMode turretVelocityMode, Datalog datalog) {

        this.telemetry = telemetry;
        this.intake = intake;
        this.keybinds = keybinds;
        this.goalPosition = goalPosition;
        this.mecanumDrive = mecanumDrive;
        this.turretVelocityMode = turretVelocityMode;
        this.datalog = datalog;

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        leftLiftWheel = hardwareMap.get(CRServo.class, "leftLiftServo");
        rightLiftWheel = hardwareMap.get(CRServo.class, "rightLiftServo");
        launchZoneLED = hardwareMap.get(Servo.class, "launchZoneLED");

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

    // Starts the flywheel
    public void loop() {
        telemetry.addData("Flywheel: ", flywheelState);
        telemetry.addData("Lift: ", liftWheelState);
        telemetry.addData("Turret: ", turretLockingState);
        turnOnLEDs();
        // State machine for the FLY wheels
        switch (flywheelState) {
            case ON:
                if (keybinds.flywheelWasPressed()) {
                    flywheelState = FlywheelState.OFF;
                }
                break;
            case OFF:
                leftFlywheel.setVelocity(0);
                rightFlywheel.setVelocity(0);
                if (keybinds.flywheelWasPressed()) {
                    flywheelState = FlywheelState.ON;
                }
        }
        // State machine for the LIFT wheels
        switch (liftWheelState) {
            case ON:
//                datalog.shootCycleStartTime.set(System.currentTimeMillis());
//                datalog.artifactCounter.set(artifactsWhenCrossWasPressed);

                leftLiftWheel.setPower(1.0);
                rightLiftWheel.setPower(1.0);
                if (liftWheelTimer.seconds() > InputValues.LIFT_WHEEL_WAIT_SECONDS * artifactsWhenCrossWasPressed) {
                    liftWheelState = LiftWheelState.OFF;
                }
                break;
            case OFF:
                //datalog.shootCycleEndTime.set(System.currentTimeMillis());
                leftLiftWheel.setPower(-1.0);
                rightLiftWheel.setPower(-1.0);
                if (keybinds.liftWheelWasPressed()) { // && isInLaunchZone()
                    artifactsWhenCrossWasPressed = intake.getNumberOfArtifacts();
                    liftWheelTimer.reset();
                    liftWheelState = LiftWheelState.ON;
                }
                //datalog.writeLine();
        }

        // State machine for turret locking state: Auto or Manual
        switch (turretLockingState) {
            case AUTO:
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                adjustTurret();
                if (keybinds.turretLockingStateWasPressed()) {
                    turretLockingState = TurretLockingState.MANUAL;
                }
                break;
            case MANUAL:
                turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (keybinds.turretManualAdjustmentLeftBumperIsPressed()) {
                    turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    turretMotor.setPower(InputValues.FLYWHEEL_SLOPE);
                }
                else if (keybinds.turretManualAdjustmentRightBumperIsPressed()) {
                    turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    turretMotor.setPower(InputValues.FLYWHEEL_SLOPE);
                }
                else if (keybinds.turretManualAdjustmentLeftJoystickX() != 0) {
                    double turretJoystickPower = keybinds.turretManualAdjustmentLeftJoystickX();
                    turretMotor.setPower(turretJoystickPower);
                }
                else {
                    turretMotor.setPower(0);
                }
                if (keybinds.turretLockingStateWasPressed()) {
                    turretLockingState = TurretLockingState.AUTO;
                }
                break;
        }

        // State machine to test flywheel velocity or have the velocity be based on the piecewise function
        switch (turretVelocityMode) {
            case AUTO:
                break;
            case MANUAL:
                telemetry.addData("Current velocity is: ", manualVelocity);

                if (keybinds.turretManualVelocityIncreaseWasPressed()) {
                    manualVelocity = manualVelocity + InputValues.VELOCITY_ADJUSTMENT;
                    leftFlywheel.setVelocity(manualVelocity);
                    rightFlywheel.setVelocity(manualVelocity);
                }
                if (keybinds.turretManualVelocityDecreaseWasPressed()) {
                    manualVelocity = manualVelocity - InputValues.VELOCITY_ADJUSTMENT;
                    leftFlywheel.setVelocity(manualVelocity);
                    rightFlywheel.setVelocity(manualVelocity);
                }
                break;
        }
    }

    // If x is greater than 48
        // y = x + 48, lower launch zone line formula
        // y bottom is equal to negative x plus 48
        // y top is equal to x minus 48
        // If robot y is in between y top on y bottom, the robot is in the lower launch zone
    // If x is less than 0
        // y = x, upper launch zone line formula
        // y bottom is equal to the x position of the robot
        // y top is equal to the negative x position of the robot
        // If robot y is in between y top and y bottom, the robot is in the upper launch zone
    // 0 through 48 = not inside any launch zone

    public boolean isInLaunchZone() {
        boolean inLaunchZone = false;
        mecanumDrive.updatePoseEstimate();
        Pose2d robotPose = mecanumDrive.localizer.getPose();
        if (robotPose.position.x > 48) {
            double lowerLaunchZoneTopY = robotPose.position.x - 48;
            double lowerLaunchZoneBottomY = -1 * robotPose.position.x + 48;
            if (robotPose.position.y <= lowerLaunchZoneTopY || robotPose.position.y >= lowerLaunchZoneBottomY) {
                inLaunchZone = true;
            }
        }
        if (robotPose.position.x < 0) {
            double upperLaunchZoneTopY = -1 * robotPose.position.x;
            double upperLaunchZoneBottomY = robotPose.position.x;
            if (robotPose.position.y <= upperLaunchZoneTopY || robotPose.position.y >= upperLaunchZoneBottomY) {
                inLaunchZone = true;
            }
        }
        return inLaunchZone;
    }

    public void turnOnLEDs() {
        // Controls the color of the LED that represents how many artifacts you have
        if (isInLaunchZone()) {
            launchZoneLED.setPosition(InputValues.PINK);
        }
        else {
            launchZoneLED.setPosition(InputValues.OFF);
        }
    }

    public void shoot() {
        artifactsWhenShooting = intake.getNumberOfArtifacts();
        telemetry.addData("Artifacts when shooting: ", artifactsWhenShooting);
        leftLiftWheel.setPower(1.0);
        rightLiftWheel.setPower(1.0);
        Helper.sleep(InputValues.LIFT_WHEEL_WAIT_MILLISECONDS * artifactsWhenShooting);
        leftLiftWheel.setPower(-1.0);
        rightLiftWheel.setPower(-1.0);
    }

    public double getFlywheelVelocity(double distanceToGoal) {
        double[] D = {         60,   72,   84,  144,  156,  168,  180};
        double[] velocity = {1440, 1460, 1470, 1475, 1421, 1680, 1695};
        int above_index = 0;
        int below_index = 0;


        for(int index = 0; index < D.length; index ++) {
            if (D[index] > distanceToGoal) {
                above_index = index;
                below_index = index - 1;
                break;
            }
        }

        // Choose whichever index where you are less than it but greater than the last one
        // If you are less than


        // Clamp to valid range for interpolation
        if (below_index <= 0) {
            below_index = 0;
            above_index = 1;
        } else if (above_index >= D.length - 1) {
            // If we're at or beyond the last point, just return the last velocity
            return velocity[velocity.length - 1];
        }

        double x0 = D[below_index];
        double x1 = D[above_index];
        double v0 = velocity[below_index];
        double v1 = velocity[above_index];

        // Fraction between the two distance points
        double t = (v1 - v0) / (x1 - x0);  // should be between 0 and 1

        // Proper linear interpolation: v = v0 + t * (v1 - v0)
        double flywheelV = v0 + t * (distanceToGoal - x0);

        telemetry.addData("flywheel velocity", flywheelV);
        telemetry.addData("velocity index", v0);
        telemetry.addData("distance index", x0);
        telemetry.addData("index", above_index);
        telemetry.addData("distance from goal", distanceToGoal);

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

        turretX = robotX + turretOffsetXField;
        turretY = robotY + turretOffsetYField;

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

        if (turretVelocityMode == TurretVelocityMode.AUTO) {
            leftFlywheel.setVelocity(flywheelVelocity);
            rightFlywheel.setVelocity(flywheelVelocity);
        }

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


