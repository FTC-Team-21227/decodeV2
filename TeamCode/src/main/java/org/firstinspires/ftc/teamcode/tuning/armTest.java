package org.firstinspires.ftc.teamcode.tuning;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

@TeleOp(name = "Arm Test")
// Program used to test hood positions
public class armTest extends LinearOpMode {
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    Arm hood;
 // 0.27 close 0.04 open
    double HoodAngle=0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) { // Lowest angle
                HoodAngle = 0;
            }
            if (gamepad1.yWasPressed()) { // Highest angle (41 degrees)
                HoodAngle = 1;
            }
            if (gamepad1.dpadUpWasPressed()) { // Increase hood position
                HoodAngle += 0.01;
            }
            if (gamepad1.dpadDownWasPressed()) { // Decrease hood position
                HoodAngle -= 0.01;
            }
//            hood.turnToAngle(HoodAngle);
            hood.ARM.setPosition(HoodAngle);
            // Telemetry lines
            telemetry.addData("Hood target angle", HoodAngle);
//            telemetry.addData("Hood curr angle", hood.getAngle());
            telemetry.addData("Hood pos", hood.ARM.getPosition());
            telemetry.addData("outside range", hood.ARM.commandedOutsideRange());
            telemetry.update();
        }
    }

    private void initialization() {
        hood = new Arm(hardwareMap);
        HoodAngle = 0;
    }
}
