package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake_Old;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

@TeleOp(name = "Full Intake + Shooter Test No Turret")
public class fullIntakeShooterTest_NoTurret extends LinearOpMode {

    Flywheel flywheel;
    Arm arm;
    Stopper stopper;
    Intake_Old intake;
    // Editable in dashboard, distance and height difference from robot to goal
    public static double distance;
    public static double heightDiff;

    double flywheelVelocity;
    double HoodPosition;


    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry (FtcDashboard.getInstance().getTelemetry());
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake_Old(hardwareMap);
        arm = new Arm(hardwareMap);
        stopper = new Stopper(hardwareMap);
//        hood = hardwareMap.get(Servo.class, "hood");
        //        initialization();
        waitForStart();
        double hoodPos = 0.8;
        flywheelVelocity = 0;
        arm.ARM.setPosition(0.8);
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
            if (gamepad1.bWasPressed()){
                stopper.close();
            }
            else if (gamepad1.yWasPressed()){
                stopper.open();
            }
            if (gamepad1.dpadRightWasPressed()){
                hoodPos+= 0.01;
                arm.ARM.setPosition(hoodPos);
            }
            else if (gamepad1.dpadLeftWasPressed()){
                hoodPos -= 0.01;
                arm.ARM.setPosition(hoodPos);
            }
            //arm pos 0.74
            flywheel.spinTo(flywheelVelocity);
            telemetry.addData("Flywheel Target Vel", flywheelVelocity);
            telemetry.addData("Flywheel Current Vel", flywheel.getVel());
            telemetry.addData("arm pos", hoodPos);
            telemetry.update();

        }
    }

};