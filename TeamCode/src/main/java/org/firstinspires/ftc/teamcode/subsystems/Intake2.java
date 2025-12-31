package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot2;

import java.util.concurrent.TimeUnit;

public class Intake2 {
    DcMotor INTAKE;
    double voltageComp = 1;
    ElapsedTime timer = new ElapsedTime();

    public Intake2(HardwareMap hardwareMap){
        INTAKE = hardwareMap.get(DcMotor.class, "intake");
        INTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
        INTAKE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        INTAKE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        INTAKE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake(double k) {INTAKE.setPower(k * Robot.Constants.intakePower);}

    public void intake(double k, double runTime) {
        timer.reset();
        while (timer.time(TimeUnit.SECONDS) < runTime) {intake(k);}
        stop();
        timer.reset();
    }

    public void outtake(double k) {INTAKE.setPower(-1 * k * Robot.Constants.outtakePower);}

    public void outtake(double k, double runTime) {
        timer.reset();
        while (timer.time(TimeUnit.SECONDS) < runTime) {outtake(k);}
        stop();
        timer.reset();
    }

    public void stop() {INTAKE.setPower(0);}

    public void smallIntake() {intake(Robot2.Constants.slowIntakePower, Robot2.Constants.intakePulseTime);}

    public void smallOuttake() {outtake(Robot2.Constants.outtakePower, Robot2.Constants.outtakePulseTime);}
}
