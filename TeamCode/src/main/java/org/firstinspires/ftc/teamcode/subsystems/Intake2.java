package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.Constants;

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

    public void intake(double k, double runTime) { //these won't work because the loop will get stuck inside
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

    public void shootArtifact() {intake(Constants.INTAKE_POWER, Constants.INTAKE_SHOOT_TIME);}

    public void nextArtifact() {intake(Constants.INTAKE_POWER, Constants.NEXT_ARTIFACT_TIME);}

    public void smallOuttake() {outtake(Constants.OUTTAKE_POWER, Constants.OUTTAKE_PULSE_TIME);}
}
