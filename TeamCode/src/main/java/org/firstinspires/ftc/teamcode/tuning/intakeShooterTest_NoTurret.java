package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel_Old;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake_Old;

@TeleOp(name = "Intake + Shooter Test No Turret")
public class intakeShooterTest_NoTurret extends LinearOpMode {

    Flywheel_Old flywheel;
    Servo hood;
    Intake_Old intake;
    // Editable in dashboard, distance and height difference from robot to goal
    public static double distance;
    public static double heightDiff;

    double flywheelVelocity;
    double HoodPosition;


    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry (FtcDashboard.getInstance().getTelemetry());
        flywheel = new Flywheel_Old(hardwareMap);
        intake = new Intake_Old(hardwareMap);
//        hood = hardwareMap.get(Servo.class, "hood");
        //        initialization();
        waitForStart();
        double hoodPos = 0.5;
        flywheelVelocity = 0;
        while (opModeIsActive()) {
//            if (gamepad1.left_bumper){ // Move hood down
//                hoodPos += 0.001;
//                hood.setPosition(hoodPos);
//            }
//            if (gamepad1.right_bumper){ // Move hood up
//                hoodPos -= 0.001;
//                hood.setPosition(hoodPos);
//            }
            if (gamepad1.dpad_up){
                flywheelVelocity += 5;
            }
            if (gamepad1.dpad_down){
                flywheelVelocity -= 5;
            }
            if (gamepad1.a){
                intake.intake();
            }
            else if (gamepad1.x){
                intake.stop();
            }
            flywheel.spinTo(flywheelVelocity);
            telemetry.addData("Flywheel Target Vel", flywheelVelocity);
            telemetry.addData("Flywheel Current Vel", flywheel.getVel());
//            telemetry.addData("hood pos", hood.getPosition());
            telemetry.update();

        }
    }

};