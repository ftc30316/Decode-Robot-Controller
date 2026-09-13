package org.firstinspires.ftc.teamcode.biobuzz;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.DouglasFIRST;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.InputValues;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Keybinds;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Turret;

public class Robot {

    private MecanumDrive mecanumDrive;
    private IntakeComponent intake;
    private HardwareMapping hardwareMapping;
    private Config config;
    public Robot (HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Pose2d beginPose){
        this.config = new Config();
        this.hardwareMapping = new HardwareMapping(config, hardwareMap);
        Keybinds keybinds = new Keybinds(gamepad1, gamepad2);
        this.mecanumDrive = new MecanumDrive(hardwareMap, keybinds, beginPose);
        this.intake = new IntakeComponent(config, hardwareMapping, keybinds);
    }

    public void init(){
        DcMotorEx OuttakeFlywheelMotor = this.hardwareMapping.getOuttakeFlywheelMotor();
        OuttakeFlywheelMotor.setVelocity(config.OuttakeFlywheelMotorDefaultVelocity);
    }
    public void loop() {
        DcMotorEx OuttakeFlywheelMotor = this.hardwareMapping.getOuttakeFlywheelMotor();

        Pose2d currentPose = getCurrentPose();
//        telemetry.addData("Robot X", currentPose.position.x);
//        telemetry.addData("Robot Y", currentPose.position.y);
//        telemetry.addData("Robot Heading", Math.toDegrees(currentPose.heading.toDouble()));
//        telemetry.addData("Robot speed", mecanumDrive.drivePowers);

        mecanumDrive.loop();
        intake.loop();
    }
    public Pose2d getCurrentPose() {
        mecanumDrive.updatePoseEstimate();
        return mecanumDrive.localizer.getPose();
    }
}
