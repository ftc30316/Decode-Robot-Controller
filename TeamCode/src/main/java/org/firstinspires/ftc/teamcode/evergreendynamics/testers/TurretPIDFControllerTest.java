package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

@TeleOp(name="Tune: Turret PID", group = "Concept")
@Config
public class TurretPIDFControllerTest extends LinearOpMode {

    // our DC motor.
    DcMotorEx turretMotor;
    DcMotorEx rightFlywheel;

    public static double NEW_P = 15;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;
    public static double NEW_F = 0.0;

    public void runOpMode() {
        // get reference to DC motor.
        // since we are using the Control Hub or Expansion Hub,
        // cast this motor to a DcMotorEx object.
        turretMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "turretMotor");
        //rightFlywheel = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightFlywheel");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.9);

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        // wait for start command.
        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidfOrig = turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfModified = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        //turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // re-read coefficients and verify change.
        //PIDFCoefficients pidfModified = pidfNew; //turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        int lowPosition = 0;
        int highPosition = 1000;
        int targetPosition = lowPosition;

        // display info to user.
        while(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.0f, %.02f, %.02f, %.02f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.02f, %.02f, %.02f, %.02f",
                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);

            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", turretMotor.getVelocity());

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Position", targetPosition);
            packet.put("Left Current Position", turretMotor.getVelocity());
            //packet.put("Right Current Velocity", rightFlywheel.getVelocity());

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if (gamepad1.crossWasPressed()) {
                pidfModified = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
                turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfModified);
                //rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfModified);
            }

            if (gamepad1.dpadDownWasPressed()) {
                targetPosition = lowPosition;
            }

            if (gamepad1.dpadUpWasPressed()) {
                targetPosition = highPosition;
            }

            turretMotor.setTargetPosition(targetPosition);
            //rightFlywheel.setVelocity(targetPosition);

            telemetry.update();
        }
    }
}