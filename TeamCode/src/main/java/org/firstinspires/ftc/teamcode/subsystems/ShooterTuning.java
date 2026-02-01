package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "stuner")
@Config
public class ShooterTuning extends OpMode {

    public static double HOOD_ANGLE_DEG = 0;     // degrees
    public static double FLYWHEEL_RPM = 0;     // rpm

    Shooter shooter;
    Limelight limelight;
//    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {

        // Convert to radians
        double hoodAngleRad = Math.toRadians(HOOD_ANGLE_DEG);

        if (gamepad1.a) {
            // Spin flywheel
            shooter.flywheel.spinTo(FLYWHEEL_RPM);

            // Move hood + turret to test angles
            shooter.getHood().turnToAngle(hoodAngleRad);
        } else {
            shooter.flywheel.setPower(0);
        }

        telemetry.addData("target hood angle", HOOD_ANGLE_DEG);
        telemetry.addData("cur hood angle", Math.toDegrees(shooter.getHood().getAngle()));

        telemetry.addData("target flywheel speed", FLYWHEEL_RPM);
        telemetry.addData("cur flywheel speed", shooter.flywheel.getVel());

        telemetry.addData("lmlight distance to goal", limelight.getDistanceToGoal()); // should be inches
        // offsetted to center of robot

        telemetry.update();
    }
}