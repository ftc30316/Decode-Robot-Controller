package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Keybinds;

public class OuttakeComponent {

    private Config config;
    private HardwareMapping hardwareMapping;
    private Keybinds keybinds;
    private OuttakeState outtakeState;
    public OuttakeComponent(Config config, HardwareMapping hardwareMapping, Keybinds keybinds) {
        this.config = config;
        this.hardwareMapping = hardwareMapping;
        this.keybinds = keybinds;
        this.outtakeState = config.OuttakeDefaultState;
    }

    public void loop() {

        if (keybinds.changeOuttakeState()) {
            CRServo OuttakeServo = this.hardwareMapping.getOuttakeServo();
            DcMotorEx OuttakeFlywheelMotor = this.hardwareMapping.getOuttakeFlywheelMotor();
            switch (outtakeState) {
                case OFF:
                    OuttakeServo.setPower(1);
                    OuttakeFlywheelMotor.setVelocity(config.OuttakeFlywheelMotorDefaultVelocity);
                    outtakeState = OuttakeState.ON;
                    break;
                case ON:
                    OuttakeServo.setPower(0);
                    OuttakeFlywheelMotor.setVelocity(0);
                    outtakeState = OuttakeState.OFF;
                    break;
            }
        }

    }
}
