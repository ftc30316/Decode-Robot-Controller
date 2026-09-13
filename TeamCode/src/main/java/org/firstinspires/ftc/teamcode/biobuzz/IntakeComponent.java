package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Keybinds;

public class IntakeComponent {

    private Config config;
    private HardwareMapping hardwareMapping;
    private Keybinds keybinds;
    private IntakeState intakeState;
    public IntakeComponent(Config config, HardwareMapping hardwareMapping, Keybinds keybinds) {
        this.config = config;
        this.hardwareMapping = hardwareMapping;
        this.keybinds = keybinds;
        this.intakeState = config.IntakeDefultState;
    }

    public void loop() {

        if (keybinds.changeIntakeState()) {
            CRServo IntakeServo = this.hardwareMapping.getIntakeServo();
            DcMotorEx IntakeSweeperMotor = this.hardwareMapping.getIntakeSweeperMotor();
            switch (intakeState) {
                case OFF:
                    IntakeServo.setPower(0);
                    IntakeSweeperMotor.setVelocity(0);
                    intakeState = IntakeState.ON;
                    break;
                case ON:
                    IntakeServo.setPower(1);
                    intakeState = IntakeState.OFF;
                    IntakeSweeperMotor.setVelocity(config.IntakeDefaultSweeperMotorVelocity);
                    break;
            }
        }

    }
}
