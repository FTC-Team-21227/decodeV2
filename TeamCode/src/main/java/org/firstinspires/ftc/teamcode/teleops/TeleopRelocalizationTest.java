package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class TeleopRelocalizationTest extends OpMode {
    Robot robot;
    boolean moveShot = false;
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(),telemetry);
    public void init(){
        robot = Robot.startInstance(new Pose(72,72, 0), Robot.Color.RED); //start facing the goals, RED poses
        robot.initFollower(hardwareMap, joinedTelemetry);
//        robot.turret.turnToRobotAngle(0);
//        robot.hood.turnToAngle(Math.toRadians(45));
        joinedTelemetry.update();
    }
//    public void start(){
//        robot.startTeleop();
//    }
//    double turretPos = 0.5;
    public void loop(){
//        if (gamepad1.dpad_up){
//            turretPos += 0.001;
//            robot.turret.turret.setPosition(turretPos);
//        }
//        if (gamepad1.dpad_down){
//            turretPos -= 0.001;
//            robot.turret.turret.setPosition(turretPos);
//        }
        //toggles: LB, x, y
        if (gamepad1.aWasPressed()) moveShot = !moveShot;
//        robot.updateVoltage(telemetry);
        //final: back = relocalize
        robot.updateFollower(gamepad1.backWasPressed(), gamepad1.touchpadWasPressed(), telemetry);
        //final version will be: RT = front feeder, LT = back feeder, RB = alternating feeder, x = toggle setPose => shooter lock/manual control, rightstick up/down = flywheel scale, dpad up/down= hood, dpad left/right = turret, gamepad2 y = human feed toggle, start = power flywheel off, gamepad2 bumpers and triggers = feeders manual control, gamepad2 a = velocity correction
        robot.calculateShooter(joinedTelemetry,moveShot);
        joinedTelemetry.update();
    }
//    public void stop(){
//        robot.setPose(robot.drive2.localizer.getPose());
//    }
}
