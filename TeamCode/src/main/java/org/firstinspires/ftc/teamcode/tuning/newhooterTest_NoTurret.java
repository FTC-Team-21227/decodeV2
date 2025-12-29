package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel_Old;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

@TeleOp(name = "New Shooter Test No Turret")
public class newhooterTest_NoTurret extends LinearOpMode {

    Flywheel flywheel;
    Hood hood;
    // Editable in dashboard, distance and height difference from robot to goal
    public static double distance;
    public static double heightDiff;

    double flywheelVelocity;
    double HoodPosition;


    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry (FtcDashboard.getInstance().getTelemetry());
        flywheel = new Flywheel(hardwareMap);
//        hood = hardwareMap.get(Servo.class, "hood");
        //        initialization();
        waitForStart();
        double hoodPos = 0.5;
        flywheelVelocity = 0;
        while (opModeIsActive()) {
            if (gamepad1.left_bumper){ // Move hood down
                hoodPos += 0.001;
            }
            if (gamepad1.right_bumper){ // Move hood up
                hoodPos -= 0.001;
            }
            if (gamepad1.dpad_up){
                flywheelVelocity += 5;
            }
            if (gamepad1.dpad_down){
                flywheelVelocity -= 5;
            }
            if (gamepad1.b){
                flywheelVelocity = 0;
                flywheel.FLYWHEEL_MASTER.setPower(0);
                flywheel.FLYWHEEL2.setPower(0);
            }
            else flywheel.spinTo(flywheelVelocity);
            hood.HOOD.setPosition(hoodPos);
            telemetry.addData("Flywheel Target Vel", flywheelVelocity);
            telemetry.addData("Flywheel Current Vel", flywheel.getVel());
            telemetry.addData("Hood target pos", hoodPos);
            telemetry.addData("Hood curr angle", hood.getAngle());
            telemetry.addData("Hood pos", hood.HOOD.getPosition());
            telemetry.addData("outside range", hood.HOOD.commandedOutsideRange());
            telemetry.update();
//            telemetry.addData("hood pos", hood.getPosition());
            telemetry.update();

        }
    }

};