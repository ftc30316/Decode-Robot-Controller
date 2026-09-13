package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMapping {
    private final Config config;
    private final HardwareMap hardwareMap;
    public HardwareMapping (Config config, HardwareMap hardwareMap){
        this.config = config;
        this.hardwareMap = hardwareMap;
    }

    public CRServo getIntakeServo() {
        return hardwareMap.get(CRServo.class, config.IntakeServoName);
    }

    public DcMotorEx getIntakeSweeperMotor() {
        return hardwareMap.get(DcMotorEx.class, config.IntakeSweeperMotorName);
    }
    public CRServo getOuttakeServo() {
        return hardwareMap.get(CRServo.class, config.OuttakeServoName);
    }

    public DcMotorEx getOuttakeFlywheelMotor() {
        return hardwareMap.get(DcMotorEx.class, config.OuttakeFlywheelMotorName);
    }
}
