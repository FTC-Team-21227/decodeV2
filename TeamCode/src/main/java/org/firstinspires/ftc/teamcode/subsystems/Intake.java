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
    CachedMotor INTAKE2;
    double power = 0;
    boolean paused = false;
    double voltageComp = 1;
    //power
    public Intake(HardwareMap hardwareMap){
        INTAKE = new CachedMotor(hardwareMap.get(DcMotorEx.class, "intake"));
        INTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
        INTAKE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        INTAKE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        INTAKE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        INTAKE2 = new CachedMotor(hardwareMap.get(DcMotorEx.class, "intake2"));
        INTAKE2.setDirection(DcMotorSimple.Direction.REVERSE);
        INTAKE2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        INTAKE2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        INTAKE2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    // Modes
    public void intake(){
        if (!paused) {INTAKE.setPower(1* Robot.Constants.intakePower); INTAKE2.setPower(1* Robot.Constants.intakePower);}
        power = 1*Robot.Constants.intakePower;
    }
    public void slowIntake(){
        if (!paused) {INTAKE.setPower(1*Robot.Constants.slowIntakePower); INTAKE2.setPower(1* Robot.Constants.slowIntakePower);}
        power = 1*Robot.Constants.slowIntakePower;
    }
    public void outtake(){
        if (!paused) {INTAKE.setPower(-1*Robot.Constants.outtakePower); INTAKE2.setPower(1* Robot.Constants.outtakePower);}
        power = -1*Robot.Constants.outtakePower;
    }
    public void strongOuttake(){
        if (!paused) {INTAKE.setPower(-1); INTAKE2.setPower(-1);}
        power = -1;
    }
    public void stop(){
        if (!paused) {INTAKE.setPower(0); INTAKE2.setPower(-1);}
        power = 0;
    }
    // Start and stop
    public void pause(){
        INTAKE.setPower(0);
        INTAKE2.setPower(0);
        paused = true;
    }
    public void shortOuttake(){
        if (!paused) {INTAKE.setPower(-1*Robot.Constants.outtakePower); INTAKE2.setPower(-1*Robot.Constants.outtakePower);}
        paused = true;
    }
    public void proceed() {
        INTAKE.setPower(power * voltageComp);
        INTAKE2.setPower(power * voltageComp);
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