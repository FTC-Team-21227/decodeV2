package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedServo;

//two CR axons and 1 analog input
public class Turret {
    public RTPAxon turret;
    public CRServo turret2; //make this cached later
    public static double current_angle = 0;

    public Turret(HardwareMap hardwareMap){
        turret = new RTPAxon(hardwareMap.get(CRServo.class,"turret"), hardwareMap.get(AnalogInput.class, "turretEncoder")); //make it cached servo later
        turret2 = hardwareMap.get(CRServo.class, "turret2");
//        turret.scaleRange(0,1); // 0 = +90 deg, 1 = -330 deg; DOUBLE CHECK
    }

    // Turns turret to the robot-relative angle in degrees
    public void turnToRobotAngle(double angle) {
        angle = AngleUnit.normalizeDegrees(angle- Robot.Constants.turretTargetRangeOffset) + Robot.Constants.turretTargetRangeOffset;
//        turret.setTargetRotation(constrain(Range.scale(angle,Robot.Constants.turretLowAngle,Robot.Constants.turretHighAngle,Robot.Constants.turretScale0,Robot.Constants.turretScale1)));
//        double thing = constrain(angle - Robot.Constants.turretAngleOffset)/Robot.Constants.turretGearRatio;
//        RobotLog.a("" + thing);
        turret.setTargetRotation(constrain(angle - Robot.Constants.turretAngleOffset)/Robot.Constants.turretGearRatio);
        turret.update();
        turret2.setPower(turret.getPower());
        current_angle = turret.totalRotation() * Robot.Constants.turretGearRatio + Robot.Constants.turretAngleOffset;
    }

    // Gives turret's robot-relative angle (in degrees)
    public double getTurretRobotAngle() {
        return turret.getTotalRotation() * Robot.Constants.turretGearRatio + Robot.Constants.turretAngleOffset;
        //current_angle;
//        return Range.scale(turret.getCurrentAngle(), Robot.Constants.turretScale0,Robot.Constants.turretScale1,Robot.Constants.turretLowAngle,Robot.Constants.turretHighAngle);
    }

    // Gives turret's robot-relative pose based on its position and current heading
    public Pose getPoseRobotTurret() {
        return new Pose(Robot.Constants.turretPos.getX(),Robot.Constants.turretPos.getY(),getTurretRobotAngle());
    }

    // Function returns the closest position that is still in range (turret will not move if not in range)
    public double constrain(double ogPos){
        return Range.clip(ogPos,Robot.Positions.turretClip0,Robot.Positions.turretClip1);
    }
}
