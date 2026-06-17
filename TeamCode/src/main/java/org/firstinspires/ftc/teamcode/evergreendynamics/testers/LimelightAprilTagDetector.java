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

            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                telemetry.addData("Field Location (m)",
                        "(%.3f, %.3f)", botpose.getPosition().x, botpose.getPosition().y);
            }

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            telemetry.addData("Tags Visible", tags.size());

            TelemetryPacket packet = new TelemetryPacket();

            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();

                // Bearing (horizontal angle) and elevation (vertical angle) to tag
                double bearing=tag.getTargetXDegrees(); // horizontal angle
                double elevation=tag.getTargetYDegrees(); //vertical angle

                // Trig distance estimate
                double angleRad = (LIMELIGHT_MOUNT_ANGLE_DEG + elevation) * (Math.PI / 180.0);
                double rangeInches = (GOAL_HEIGHT_IN - LIMELIGHT_LENS_HEIGHT_IN) / Math.tan(angleRad);

                // 3D pose relative to tag (target space): x=strafe, z=forward
                Pose3D robotPose = tag.getRobotPoseTargetSpace();
                double strafeMeters = robotPose.getPosition().x;
                double forwardMeters = robotPose.getPosition().z;

                // Field pose from this tag alone
                Pose3D fieldPose = tag.getRobotPoseFieldSpace();

                telemetry.addLine("Tag #" + id);
                telemetry.addData("  Range (in)","%.2f", rangeInches);
                telemetry.addData("  Bearing (deg)","%.2f", bearing);
                telemetry.addData("  Elevation (deg)","%.2f", elevation);
                telemetry.addData("  Strafe (m)","%.3f", strafeMeters);
                telemetry.addData("  Forward (m)","%.3f", forwardMeters);
                if (fieldPose != null) {
                    telemetry.addData("  Field pos (m)",
                            "(%.3f, %.3f)", fieldPose.getPosition().x, fieldPose.getPosition().y);
                }

                String pre = "Tag" + id + "/";
                packet.put(pre + "range_in",rangeInches);
                packet.put(pre + "bearing_deg",bearing);
                packet.put(pre + "elevation_deg",elevation);
                packet.put(pre + "strafe_m",strafeMeters);
                packet.put(pre + "forward_m",forwardMeters);
            }

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }

        limelight.stop();
    }
}
