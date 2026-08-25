package org.firstinspires.ftc.teamcode.evergreendynamics.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TrackingTurret {
    private DcMotorEx turret;
    private double kP=0.013; // need to tune
    private double kD=0.00009; // need to tune
    private double lastError=0;
    private double angleTolerance=0.2;
    private final double MAX_POWER=0.60; // need to tune, 60% for rn
    private double power=0;
    private final ElapsedTime timer=new ElapsedTime();

    public void init(HardwareMap hwMap){
        turret=hwMap.get(DcMotorEx.class,"turretMotor");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setkP(double newkP){
        kP=newkP;
    }
    public double getkP(){
        return kP;
    }

    public void setkD(double newkD){
        kD=newkD;
    }
    public double getkD(){
        return kD;
    }

    public double getPower(){
        return power;
    }

    public void resetTimer(){
        timer.reset();
    }

    public void update(double tx){
        double deltaTime=timer.seconds();
        timer.reset();

        // PD controller
        double error=-tx; // error=goal-current so error=0-tx=-tx
        double pTerm=error*kP;

        double dTerm=0;
        if(deltaTime>0){
            dTerm=((error-lastError)/deltaTime)*kD;
        }
        if(Math.abs(error)<angleTolerance){
            power=0;
        }else{
            power=Range.clip(pTerm+dTerm,-MAX_POWER,MAX_POWER);
        }

        // safety encoder check
        turret.setPower(power);
        lastError=error;
    }

    public void stop() {
        turret.setPower(0);
        lastError = 0;
    }
}
