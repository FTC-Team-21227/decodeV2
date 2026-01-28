package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedMotor;

public class Intake {
    //power
    CachedMotor INTAKE;
    CachedMotor TRANSFER;
    double power1 = 0;
    double power2 = 0;
    boolean paused = false;
    double voltageComp = 1;
    //power
    public Intake(HardwareMap hardwareMap){
        INTAKE = new CachedMotor(hardwareMap.get(DcMotorEx.class, "intake"));
        INTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
        INTAKE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        INTAKE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        INTAKE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TRANSFER = new CachedMotor(hardwareMap.get(DcMotorEx.class, "transfer"));
        TRANSFER.setDirection(DcMotorSimple.Direction.REVERSE);
        TRANSFER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TRANSFER.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TRANSFER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    // Modes
    public void intake(){
        if (!paused) {INTAKE.setPower(1* Robot.Constants.intakePower); TRANSFER.setPower(0 /*1* Robot.Constants.intakePower*/);}
        power1 = 1*Robot.Constants.intakePower;
        power2 = 0;
    }
    public void slowIntake(){
        if (!paused) {INTAKE.setPower(1*Robot.Constants.slowIntakePower); TRANSFER.setPower(0 /*1* Robot.Constants.slowIntakePower*/);}
        power1 = 1*Robot.Constants.slowIntakePower;
        power2 = 0;
    }
    public void shoot(){
        if (!paused) {INTAKE.setPower(1* Robot.Constants.intakePower); TRANSFER.setPower(1* Robot.Constants.intakePower);}
        power1 = 1*Robot.Constants.intakePower;
        power2 = 1*Robot.Constants.intakePower;
    }
    public void outtake(){
        if (!paused) {INTAKE.setPower(-1*Robot.Constants.outtakePower); TRANSFER.setPower(-1* Robot.Constants.outtakePower);}
        power1 = -1*Robot.Constants.outtakePower;
        power2 = -1*Robot.Constants.outtakePower;
    }
    public void strongOuttake(){
        if (!paused) {INTAKE.setPower(-1); TRANSFER.setPower(-1);}
        power1 = -1;
        power2 = -1;
    }
    public void stop(){
        if (!paused) {INTAKE.setPower(0); TRANSFER.setPower(0);}
        power1 = 0;
        power2 = 0;
    }
    // Start and stop
    public void pause(){
        INTAKE.setPower(0);
        TRANSFER.setPower(0);
        paused = true;
    }
    public void shortOuttake(){
        if (!paused) {INTAKE.setPower(-1*Robot.Constants.outtakePower); TRANSFER.setPower(-1*Robot.Constants.outtakePower);}
        paused = true;
    }
    public void proceed() {
        INTAKE.setPower(power1 * voltageComp);
        TRANSFER.setPower(power2 * voltageComp);
        paused = false;
    }
//    public void updateComp(){
//        voltageComp = 14.0/Flywheel.volts;
////        if (voltageComp %0.1==0){
//            RobotLog.d("comp"+voltageComp);
//            RobotLog.d("volts"+Flywheel.volts);
////        }
//    }
}