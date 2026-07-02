package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class LimelightAprilTagDetector extends LinearOpMode {

    private static final int APRILTAG_PIPELINE = 0;
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;
    private static final double LIMELIGHT_LENS_HEIGHT_IN = 20.0;
    private static final double GOAL_HEIGHT_IN = 60.0;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addData("AprilTags", "No targets detected");
                telemetry.update();
                continue;
            }

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            telemetry.addData("Tags Visible", tags.size());

            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();

                double bearing=tag.getTargetXDegrees();
                double elevation=tag.getTargetYDegrees();

                double angleRad    = (LIMELIGHT_MOUNT_ANGLE_DEG + elevation) * (Math.PI / 180.0);
                double rangeInches = (GOAL_HEIGHT_IN - LIMELIGHT_LENS_HEIGHT_IN) / Math.tan(angleRad);

                telemetry.addLine("Tag #" + id);
                telemetry.addData("  Range (in)","%.2f", rangeInches);
                telemetry.addData("  Bearing (deg)","%.2f", bearing);
                telemetry.addData("  Elevation (deg)","%.2f", elevation);
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
