package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedServo;

public class Turret {
    public CachedServo turret;

    public Turret(HardwareMap hardwareMap){
        turret = new CachedServo(hardwareMap.get(Servo.class,"turret"));
        turret.scaleRange(0,1); // 0 = +90 deg, 1 = -330 deg; DOUBLE CHECK
    }

    // Turns turret to the robot-relative angle in radians
    public void turnToRobotAngle(double angle) {
        angle = AngleUnit.normalizeRadians(angle- Robot.Constants.turretTargetRangeOffset) + Robot.Constants.turretTargetRangeOffset;
        turret.setPosition(constrain(Range.scale(angle,Robot.Constants.turretLowAngle,Robot.Constants.turretHighAngle,Robot.Constants.turretScale0,Robot.Constants.turretScale1)));
    }

    // Gives turret's robot-relative angle (in radians)
    public double getTurretRobotAngle() {
        return Range.scale(turret.getPosition(), Robot.Constants.turretScale0,Robot.Constants.turretScale1,Robot.Constants.turretLowAngle,Robot.Constants.turretHighAngle);
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
