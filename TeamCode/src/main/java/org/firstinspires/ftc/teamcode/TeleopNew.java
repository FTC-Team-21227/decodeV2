package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopNew extends OpMode {
    Robot robot;
    boolean intake = false; //false = stop, true = intake
    boolean setPose = false;
    boolean human = false;
    boolean slow = false;
    boolean p2p = false;
    boolean RT = false;
    boolean LT = false;
    boolean moveShot = false;
    boolean disableFlywheel = false;
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(),telemetry);
    public void init(){
        robot = Robot.getInstance(new Pose(0,0, Math.PI), Robot.Color.BLUE); //start facing the goals, RED poses
        robot.initTeleop(hardwareMap, telemetry);
//        robot.turret.turnToRobotAngle(0);
//        robot.hood.turnToAngle(Math.toRadians(45));
        joinedTelemetry.update();
    }
    public void start(){
        robot.startTeleop();
    }
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
        if (gamepad1.leftBumperWasPressed()) intake = !intake;
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) setPose = !setPose;
        if (gamepad2.yWasPressed()) human = !human;
        if (gamepad2.leftStickButtonWasPressed() /*|| gamepad1.leftStickButtonWasPressed()*/) slow = !slow;
//        if (gamepad1.yWasPressed()) p2p = !p2p;
        if (gamepad2.aWasPressed()) moveShot = !moveShot;
//        if (gamepad1.startWasPressed() || gamepad2.startWasPressed()) disableFlywheel = !disableFlywheel;
        RT = gamepad2.right_trigger > 0.1;
        LT = gamepad2.left_trigger > 0.1;
//        robot.updateVoltage(telemetry);
        //final: back = relocalize
        robot.updateFollower(false, gamepad1.touchpadWasPressed(), joinedTelemetry);
        //final version will be: LB = intake toggle, b = reverse, RB = shoot request
        robot.updateIntake(intake, gamepad1.b, !intake, gamepad1.right_bumper, joinedTelemetry);
        robot.setGoalTarget();
        //final version will be: RT = spinup request, LT = idle request, x = toggle setPose => shooter lock/manual control, rightstick up/down = flywheel scale, dpad up/down= hood, dpad left/right = turret, gamepad2 y = human feed toggle, start = power flywheel off, gamepad2 bumpers and triggers , gamepad2 a = velocity correction
        robot.updateShooter(joinedTelemetry,gamepad1.right_trigger > 0.1, human, setPose, gamepad1.left_trigger > 0.1, moveShot, null, gamepad1.right_stick_y + gamepad2.right_stick_y, gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed(), gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed(), gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed(), gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed());
        //final: toggle left stick button = slow mode, toggle y = p2p drive but it stops on its own
        p2p = robot.driveFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, slow, p2p);
//        robot.drive2.drawPose(packet);
//        telemetry.addLine(""+robot.turret.turret.getPosition());
//        telemetry.addLine(""+robot.drive2.localizer.getPose().position.x + ", "+robot.drive2.localizer.getPose().position.y + ", "+robot.drive2.localizer.getPose().heading.toDouble());
//        telemetry.addLine(""+robot.hood.HOOD.getPosition());
//        telemetry.addLine(""+robot.flywheel.FLYWHEEL.getMotor().getDirection());
//        telemetry.addLine(""+robot.feeder.BL_FEEDER.getPosition());
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        joinedTelemetry.update();
    }
//    public void stop(){
//        robot.setPose(robot.drive2.localizer.getPose());
//    }
}
