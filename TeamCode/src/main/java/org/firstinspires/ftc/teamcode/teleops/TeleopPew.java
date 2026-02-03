package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret2;

@TeleOp
public class TeleopPew extends OpMode {
    Turret2 turret;

//    Telemetry telemetry = dashboard.getTelemetry();
    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(),telemetry);
    public void init(){
        turret = new Turret2(hardwareMap);
//        robot.turret.turnToRobotAngle(0);
//        robot.hood.turnToAngle(Math.toRadians(45));
        joinedTelemetry.update();
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
//        if (gamepad1.leftBumperWasPressed()) intake = !intake;

//        robot.updateVoltage(telemetry);
        //final: back = relocalize
        turret.turnToRobotAngle(-44);
        //final: toggle left stick button = slow mode, toggle y = p2p drive but it stops on its own

        joinedTelemetry.update();
    }
//    public void stop(){
//        robot.setPose(robot.drive2.localizer.getPose());
//    }
}
