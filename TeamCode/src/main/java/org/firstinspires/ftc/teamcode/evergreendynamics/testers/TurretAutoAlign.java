package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
public class TurretAutoAlign extends LinearOpMode {
    private Limelight3A limelight;
    private TrackingTurret turret=new TrackingTurret();

    public static int PIPELINE=8;
    public static int TAG_ID=23;

    double[] stepSizes={0.1,0.01,0.001,0.0001,0.00001};
    int stepIndex=2;

    @Override
    public void runOpMode(){
        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(PIPELINE);
        turret.init(hardwareMap);
        telemetry.addLine("Initialized");

        waitForStart();
        turret.resetTimer();

        limelight.start();

        while(opModeIsActive()){
            // update P and D on the fly
            // 'B' button cycles through the different step sizes for tuning precision
            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            // D-pad left/right adjusts the P gain.
            if (gamepad1.dpadLeftWasPressed()) {
                turret.setkP(turret.getkP() - stepSizes[stepIndex]);
            }

            if (gamepad1.dpadRightWasPressed()) {
                turret.setkP(turret.getkP() + stepSizes[stepIndex]);
            }

            // D-pad up/down adjusts the D gain.
            if (gamepad1.dpadUpWasPressed()) {
                turret.setkD(turret.getkD() + stepSizes[stepIndex]);
            }

            if (gamepad1.dpadDownWasPressed()) {
                turret.setkD(turret.getkD() - stepSizes[stepIndex]);
            }

            telemetry.addLine("--------------------------------");
            telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", turret.getkP());
            telemetry.addData("Tuning D", "%.5f (D-Pad U/D)", turret.getkD());
            telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);

            LLResult result=limelight.getLatestResult();
            if(result!=null&&result.isValid()){
                List<LLResultTypes.FiducialResult> tags=result.getFiducialResults();
                if(tags.size()!=1||tags.get(0).getFiducialId()!=TAG_ID){
                    telemetry.addLine("No Tag Detected. Stopping Turret Motor");
                    turret.stop();
                }else{
                    double tx=tags.get(0).getTargetXDegrees();
                    telemetry.addData("Tag ID", TAG_ID);
                    telemetry.addData("TX",tx);
                    turret.update(tx);
                    telemetry.addData("Power",turret.getPower());
                }
            }else{
                telemetry.addLine("No Tag");
                turret.stop();
            }
            telemetry.update();
        }
        turret.stop();
        limelight.stop();
    }
}
