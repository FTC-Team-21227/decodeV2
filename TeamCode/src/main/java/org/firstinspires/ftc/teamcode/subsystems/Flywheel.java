package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedMotor;
import org.firstinspires.ftc.teamcode.lib.FeedforwardController;
import org.firstinspires.ftc.teamcode.lib.PIDController;

public class Flywheel {
    //pid methods
    public CachedMotor FLYWHEEL;
    double targetVel;
    double lastPower;
    PIDController pid;
    FeedforwardController feedforwardController;
    VoltageSensor voltageSensor;
    public static double volts;

    //power
    public Flywheel (HardwareMap hardwareMap){
        FLYWHEEL = new CachedMotor(hardwareMap.get(DcMotorEx.class,"flywheel"));
        FLYWHEEL.setDirection(DcMotorSimple.Direction.REVERSE);
        FLYWHEEL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FLYWHEEL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        pid = new PIDController(Robot.Constants.kP,Robot.Constants.kI,Robot.Constants.kD);
        feedforwardController = new FeedforwardController(Robot.Constants.kS,Robot.Constants.kV);
    }

    public double getTargetVel(){return targetVel;}
    // Spins flywheel to certain velocity using voltage-compensated PIDF
    public double spinTo(double vel) {
//        if (configvalues.p != FLYWHEEL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p || configvalues.f != FLYWHEEL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f)
//            FLYWHEEL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(configvalues.p, configvalues.i, configvalues.d, configvalues.f));
        targetVel = vel;
        double curVel = getVel();
        //pidf bangbang: go as fast as possible when spinning up then slow down for the last hundred ticks/s
        if (/*Math.abs(targetVel-curVel) > 300*/ true) {
            double pdds = pid.calculate(curVel, targetVel); // Power needed to reduce error
            volts = voltageSensor.getVoltage();
            FLYWHEEL.setPower((pdds + feedforwardController.calculate(targetVel)) / volts); // f is how much power needed to maintain, voltage compensated
            return pdds + feedforwardController.calculate(targetVel);
        }
        else{
            double power = Math.signum(targetVel-curVel);
            FLYWHEEL.setPower(power);
            return power;
        }
    }

    // Set flywheel power directly
    public void setPower(double power){
        if (power != lastPower){
            lastPower = power;
            FLYWHEEL.setPower(power);
        }
    }

    // Return flywheel current velocity
    public double getVel(){
        return FLYWHEEL.getMotor().getVelocity();
    }
}
