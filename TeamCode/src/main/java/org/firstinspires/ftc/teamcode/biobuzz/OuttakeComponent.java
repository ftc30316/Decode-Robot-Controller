package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.evergreendynamics.robot.Keybinds;

public class OuttakeComponent {

    private Config config;
    private HardwareMapping hardwareMapping;
    private Keybinds keybinds;
    private OuttakeState outtakeState;
    private double flywheelVelocity;
    private Telemetry telemetry;
    public OuttakeComponent(Config config, HardwareMapping hardwareMapping, Keybinds keybinds, Telemetry telemetry) {
        this.config = config;
        this.hardwareMapping = hardwareMapping;
        this.keybinds = keybinds;
        this.outtakeState = config.OuttakeDefaultState;
        this.flywheelVelocity = config.OuttakeFlywheelMotorDefaultVelocity;
        this.telemetry = telemetry;
    }

    public void loop() {

        DcMotorEx OuttakeFlywheelMotor = this.hardwareMapping.getOuttakeFlywheelMotor();

        if (keybinds.changeOuttakeState()) {
            CRServo OuttakeServo = this.hardwareMapping.getOuttakeServo();


            switch (outtakeState) {
                case OFF:
                    OuttakeServo.setPower(1);
                    //OuttakeFlywheelMotor.setVelocity(config.OuttakeFlywheelMotorDefaultVelocity);
                    outtakeState = OuttakeState.ON;
                    break;
                case ON:
                    OuttakeServo.setPower(0);
                    //OuttakeFlywheelMotor.setVelocity(0);
                    outtakeState = OuttakeState.OFF;
                    break;
            }
        }
        if (keybinds.turretManualVelocityIncreaseWasPressed()){
            flywheelVelocity += 20;
            if (flywheelVelocity > 3000){
                flywheelVelocity = 3000;
            }
        }
        if (keybinds.turretManualVelocityDecreaseWasPressed()){
            flywheelVelocity -= 20;
            if (flywheelVelocity < 0){
                flywheelVelocity = 0;
            }
        }
        OuttakeFlywheelMotor.setVelocity(flywheelVelocity);
        telemetry.addData("Current outtake velocity ", flywheelVelocity);

    }
}
