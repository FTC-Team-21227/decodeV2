package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MecanumDirection extends LinearOpMode {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    double MOTOR_POWER = 0.7;
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "W_FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "W_FR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "W_BL");
        backRightDrive = hardwareMap.get(DcMotor.class, "W_BR");
        waitForStart();

            while (opModeIsActive()) {
                telemetry.addLine("Press each button to turn on its respective motor");
                telemetry.addLine();
                telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>");
                telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
                telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
                telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
                telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");
                telemetry.addLine();

                if (gamepad1.x) {
                    frontLeftDrive.setPower(MOTOR_POWER);
                    telemetry.addLine("Running Motor: Front Left");
                } else if (gamepad1.y) {
                    frontRightDrive.setPower(MOTOR_POWER);
                    telemetry.addLine("Running Motor: Front Right");
                } else if (gamepad1.b) {
                    backRightDrive.setPower(MOTOR_POWER);
                    telemetry.addLine("Running Motor: Rear Right");
                } else if (gamepad1.a) {
                    backLeftDrive.setPower(MOTOR_POWER);
                    telemetry.addLine("Running Motor: Rear Left");
                } else {
                    frontLeftDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backRightDrive.setPower(0);
                    telemetry.addLine("Running Motor: None");
                }

                telemetry.update();
            }
        }
    }

