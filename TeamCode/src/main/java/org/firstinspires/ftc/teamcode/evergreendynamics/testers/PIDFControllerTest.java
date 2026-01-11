package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Created by tom on 9/26/17.
 * This assumes that you are using a REV Robotics Control Hub or REV Robotics Expansion Hub
 * as your DC motor controller.  This op mode uses the extended/enhanced
 * PID-related functions of the DcMotorEx class.
 */

@TeleOp(name="Concept: Change PID", group = "Concept")
@Config
public class PIDFControllerTest extends LinearOpMode {

    // our DC motor.
    DcMotorEx leftFlywheel;
    DcMotorEx rightFlywheel;

    public static double NEW_P = 10.0;
    public static double NEW_I = 3.0;
    public static double NEW_D = 0.0;
    public static double NEW_F = 0.0;

    public void runOpMode() {
        // get reference to DC motor.
        // since we are using the Control Hub or Expansion Hub,
        // cast this motor to a DcMotorEx object.
        leftFlywheel = (DcMotorEx)hardwareMap.get(DcMotor.class, "leftFlywheel");
        rightFlywheel = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightFlywheel");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        // wait for start command.
        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidfOrig = leftFlywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfModified = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        //turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // re-read coefficients and verify change.
        //PIDFCoefficients pidfModified = pidfNew; //turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        double targetVelocity = 500;

        // display info to user.
        while(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.0f, %.02f, %.02f, %.02f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.02f, %.02f, %.02f, %.02f",
                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);

            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", leftFlywheel.getVelocity());

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Velocity", targetVelocity);
            packet.put("Left Current Velocity", leftFlywheel.getVelocity());
            packet.put("Right Current Velocity", rightFlywheel.getVelocity());

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if (gamepad1.crossWasPressed()) {
                pidfModified = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
                leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfModified);
                rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfModified);
            }

            if (gamepad1.dpadDownWasPressed()) {
                targetVelocity = 500;
            }

            if (gamepad1.dpadUpWasPressed()) {
                targetVelocity = 1500;
            }

            leftFlywheel.setVelocity(targetVelocity);
            rightFlywheel.setVelocity(targetVelocity);

            telemetry.update();
        }
    }
}