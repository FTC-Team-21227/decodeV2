package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedServo;

public class Arm {
   public CachedServo ARM;

    public Arm(HardwareMap hardwareMap){
        ARM = new CachedServo(hardwareMap.get(Servo.class, "arm"));
        ARM.setDirection(Servo.Direction.REVERSE);
        ARM.scaleRange(0, 1); //open = , close =
    }
    public void open(){
        ARM.setPosition(Robot.Constants.armScale0);
    }
    public void close(){
        ARM.setPosition(Robot.Constants.armScale1);
    }

}
