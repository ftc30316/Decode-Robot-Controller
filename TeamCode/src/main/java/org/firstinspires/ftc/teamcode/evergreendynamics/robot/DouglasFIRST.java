package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DouglasFIRST {
    public Intake intake;
    public Turret turret;
    public Datalogger datalogger;
    public Datalog datalog;
    public Telemetry telemetry;
    public Keybinds keybinds;
    public MecanumDrive mecanumDrive;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public HardwareMap hardwareMap;

    Vector2d goalPosition = InputValues.BLUE_GOAL_POSITION;

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    Turret.TurretVelocityMode turretVelocityMode = Turret.TurretVelocityMode.AUTO;
    DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    public InputValues.Alliance alliance = InputValues.Alliance.BLUE;


    public DouglasFIRST(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Pose2d beginPose, DriveMode driveMode, Turret.TurretVelocityMode turretVelocityMode) {

        this.keybinds = new Keybinds(gamepad1, gamepad2);
        this.mecanumDrive = new MecanumDrive(hardwareMap, keybinds, beginPose);
        this.intake = new Intake(hardwareMap, keybinds, telemetry);
        this.telemetry = telemetry;
        this.turret = new Turret(hardwareMap, telemetry, keybinds, detectAlliance(hardwareMap), mecanumDrive, intake, turretVelocityMode, datalog);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.driveMode = driveMode;
        createDatalog();
    }

    public DouglasFIRST(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, InputValues.Alliance alliance, Pose2d beginPose, DriveMode driveMode, Turret.TurretVelocityMode turretVelocityMode) {
        this.mecanumDrive = new MecanumDrive(hardwareMap, keybinds, beginPose);
        this.intake = new Intake(hardwareMap, keybinds, telemetry);
        this.telemetry = telemetry;
        this.turret = new Turret(hardwareMap, telemetry, keybinds, alliance, mecanumDrive, intake, turretVelocityMode, datalog);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.driveMode = driveMode;
        createDatalog();
    }

    public void start(double robotHeadingDeg, double turretStartHeadingDeg) {
        start(robotHeadingDeg, turretStartHeadingDeg, true);
    }

    public void start(double robotHeadingDeg, double turretStartHeadingDeg, boolean useTurretAimBackgroundThread) {
        turret.initialize(robotHeadingDeg, turretStartHeadingDeg);

        // Turns on intake
        intake.turnOnIntake();

        if (useTurretAimBackgroundThread){
            //Creates background thread
            turret.createTurretBackgroundThread();
            // Intake motor starts, flywheel starts, turret starts looking for the BLUE goal
            turret.turretBackgroundThread.start();
        }
    }

    public void createDatalog() {
        Datalog datalog = new Datalog("DougData.csv");
    }
    public void shootArtifacts() {
        turret.shoot();
    }

    public void loop() {
        Pose2d currentPose = getCurrentPose();
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Drive Mode", driveMode);
        telemetry.addData("Robot X", currentPose.position.x);
        telemetry.addData("Robot Y", currentPose.position.y);
        telemetry.addData("Robot Heading", currentPose.heading.toDouble());
        telemetry.addData("Robot speed", mecanumDrive.drivePowers);
//        telemetry.addData("turret pose", getTurretPose());

        switch (driveMode) {
            case ROBOT_CENTRIC:
                setRobotCentricDrivePowers();

                if (gamepad2.xWasPressed()) {
                    driveMode = DriveMode.FIELD_CENTRIC;
                }
                break;

            case FIELD_CENTRIC:
                setFieldCentricDrivePowers();

                if (gamepad2.xWasPressed()) {
                    driveMode = DriveMode.ROBOT_CENTRIC;
                }
                break;
        }

        if (keybinds.turretAllianceChangeWasPressed()) {
            if (alliance == InputValues.Alliance.BLUE) {
                alliance = InputValues.Alliance.RED;
                turret.setAlliance(InputValues.Alliance.RED);
            } else {
                alliance = InputValues.Alliance.BLUE;
                turret.setAlliance(InputValues.Alliance.BLUE);
            }
        }

        intake.loop();
        turret.loop();
        mecanumDrive.loop();
        checkAndRunDriverShortcuts();
    }

    public void enableDemoMode() {
        turret.disableFlywheels();
        turret.disableLiftWheels();
        intake.disableIntake();
        mecanumDrive.disableDrivePowers();
    }

    public void setRobotCentricDrivePowers() {
        if (mecanumDrive.drivePowers == MecanumDrive.DrivePowers.SLOW) {
            telemetry.addData("robot speed", "SLOW");
            mecanumDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.25,
                            -gamepad1.left_stick_x * 0.25
                    ),
                    -gamepad1.right_stick_x * 0.25
            ));

        } else if (mecanumDrive.drivePowers == MecanumDrive.DrivePowers.NORMAL){
            telemetry.addData("robot speed", "FAST");
            mecanumDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }
    }

    public void setFieldCentricDrivePowers() {
        // Read pose
        mecanumDrive.updatePoseEstimate();

        double headingRad = mecanumDrive.localizer.getPose().heading.toDouble();

        double invertedHeadingRadians = -headingRad;

        float leftStickX = gamepad1.left_stick_x;
        float leftStickY = -gamepad1.left_stick_y;
        float rightStickX = -gamepad1.right_stick_x;

        if (mecanumDrive.drivePowers == MecanumDrive.DrivePowers.SLOW) {
            leftStickX *= 0.25f;
            leftStickY *= 0.25f;
            rightStickX *= 0.25f;
        }

        // If on blue alliance, gamepad values are inverted. If on red, they are normal.
        if (alliance == InputValues.Alliance.BLUE) {
            leftStickX *= -1.0f;
            leftStickY *= -1.0f;
            // invertedHeadingRadians += Math.PI;
        }

        Rotation2d invHeading = Rotation2d.fromDouble(invertedHeadingRadians);

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d fieldInput = new Vector2d(
                leftStickX,
                leftStickY
        );

        Vector2d robotInput = invHeading.times(fieldInput);

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        mecanumDrive.setDrivePowers(
                new PoseVelocity2d(
                        robotInput,
                        rightStickX
                )
        );

    }
    public Vector2d getTurretPose() {
        return new Vector2d(turret.turretX, turret.turretY);
    }

    public InputValues.Alliance detectAlliance(HardwareMap hardwareMap) {
        mecanumDrive.updatePoseEstimate();

        if (mecanumDrive.localizer.getPose().position.y < 0) {
            alliance = InputValues.Alliance.BLUE;
        } else {
            alliance = InputValues.Alliance.RED;
        }

        return alliance;
    }

    public Vector2d getParkPosition(HardwareMap hardwareMap) {
        Vector2d parkPosition = InputValues.BLUE_PARK_POSITION;

        if (alliance == InputValues.Alliance.BLUE) {
            parkPosition = InputValues.BLUE_PARK_POSITION;
            return parkPosition;
        }
        if (alliance == InputValues.Alliance.RED) {
            parkPosition = InputValues.RED_PARK_POSITION;
            return parkPosition;
        }
        return parkPosition;
    }

    public void park() {
        if (gamepad2.circleWasPressed()) {
            com.acmerobotics.roadrunner.ftc.Actions.runBlocking(getActionBuilder().setTangent(0)
                    .strafeTo(getParkPosition(hardwareMap))
                    .turnTo(0)
                    .build());
            gamepad2.rumble(5000);
        }
    }

    public void goToZero() {
        if (gamepad2.circleWasPressed()) {
            com.acmerobotics.roadrunner.ftc.Actions.runBlocking(getActionBuilder().setTangent(0)
                    .turnTo(0)
                    .build());
            gamepad2.rumble(5000);
        }
    }

    public void goTo45() {
            com.acmerobotics.roadrunner.ftc.Actions.runBlocking(getActionBuilder().setTangent(0)
                    .turnTo(-45)
                    .build());
    }

    public void checkAndRunDriverShortcuts() {
        park();
        goToZero();
    }

    public InputValues.Alliance getAlliance() {
        return alliance;
    }

    public TrajectoryActionBuilder getActionBuilder(Pose2d beginPose) {
        return mecanumDrive.actionBuilder(beginPose);
    }

    public TrajectoryActionBuilder getActionBuilder() {
        return mecanumDrive.actionBuilder(getCurrentPose());
    }

    public Pose2d getCurrentPose() {
        mecanumDrive.updatePoseEstimate();
        return mecanumDrive.localizer.getPose();
    }

    public void savePose() {
        mecanumDrive.updatePoseEstimate();
        PoseStorage.savePose(hardwareMap.appContext, mecanumDrive.localizer.getPose(), turret.getTurretDegrees());
    }

}
