package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;

import java.util.List;


@Config
@TeleOp
public class TurretAutoAimTest extends LinearOpMode {

    public static int PIPELINE = 8;

    public static int TAG_ID = 23;

    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_HEADING_DEG = 0;

    public static double MOUNT_ANGLE_DEG = 40.0;
    public static double LENS_HEIGHT_IN = 13.5; // camera height off the floor
    public static double GOAL_HEIGHT_IN = 60.0; // tag height off the floor

    public static long MAX_STALENESS_MS = 100;

    public static double TURRET_POWER = 0.4;
    public static double MAX_TURRET_ANGLE_DEG = 90.0;

    private Limelight3A limelight;
    private MecanumDrive drive;
    private DcMotorEx turretMotor;

    private boolean tagVisible = false;
    private double tagBearingDeg = 0;
    private double tagRangeIn = 0;
    private double tagTx = 0;
    private double tagTy = 0;
    private double tagTa = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap,
                new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE);
        limelight.setPollRateHz(100);

        // Assumes the turret is physically zeroed
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        forceTurretZero();
        // how aggressively it chases its target
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(InputValues.TURRET_P, 0, 0, 0));

        waitForStart();

        forceTurretZero();

        limelight.start();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            updateTagTracking();

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x));

            if (tagVisible) {
                double clampedAngleDeg = Math.max(-MAX_TURRET_ANGLE_DEG,
                        Math.min(MAX_TURRET_ANGLE_DEG, tagBearingDeg));
                int targetTicks = (int) Math.round(clampedAngleDeg * InputValues.TICKS_PER_DEGREE);
                turretMotor.setTargetPosition(targetTicks);
                turretMotor.setPower(TURRET_POWER);
            }

            telemetry.addData("Turret ticks (target/current)",
                    "%d / %d", turretMotor.getTargetPosition(), turretMotor.getCurrentPosition());
            telemetry.addData("Tag visible", tagVisible);
            if (tagVisible) {
                telemetry.addData("tx/ty/ta", "%.2f / %.2f / %.2f", tagTx, tagTy, tagTa);
                telemetry.addData("Target distance (in)", tagRangeIn);
                telemetry.addData("Turret angle rel. robot (deg)", tagBearingDeg);
            }
            telemetry.addData("Robot X", pose.position.x);
            telemetry.addData("Robot Y", pose.position.y);
            telemetry.addData("Robot Heading", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            canvas.setStroke("#3F51B5");
            Drawing.drawRobot(canvas, pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        limelight.stop();
    }

    private void forceTurretZero() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_POWER);
    }

    private void updateTagTracking() {
        tagVisible = false;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags.size() != 1 || tags.get(0).getFiducialId() != TAG_ID) {
            return;
        }
        if (result.getStaleness() > MAX_STALENESS_MS) {
            return;
        }

        LLResultTypes.FiducialResult tag = tags.get(0);
        tagTx = tag.getTargetXDegrees();
        tagTy = tag.getTargetYDegrees();
        tagTa = tag.getTargetArea();

        double angleRad = Math.toRadians(MOUNT_ANGLE_DEG + tagTy);
        tagRangeIn = (GOAL_HEIGHT_IN - LENS_HEIGHT_IN) / Math.tan(angleRad);
        tagBearingDeg = tagTx;
        tagVisible = true;
    }
}
