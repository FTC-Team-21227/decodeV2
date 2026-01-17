package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class Limelight {
    Limelight3A limelight;
    IMU imu;

    public Limelight(HardwareMap hardwareMap) // initialization constructor
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // currently tuned pipeline
        limelight.setPollRateHz(100); // update data 100 times/s
        limelight.start(); // starts limelight
    }

    // Returns camera's field-relative position
    public Pose update(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // horizontal offset (deg)
            double ty = result.getTy(); // vertical offset (deg)

            Pose3D botpose = result.getBotpose();
            // pose3d, field relative, coords + orientation

            // Extract 2D pose
            double x = botpose.getPosition().x;      // field X
            double y = botpose.getPosition().y;      // field Y
            double heading = botpose.getOrientation().getYaw(); // robot heading in radians

            return new Pose(x, y, heading);
        } else {
            telemetry.addData("Limelight", "No Targets");
            return null; // no target
        }
    }

    public double getTx() // degrees left or right of target
    {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            telemetry.addData("Target X", tx);
            return tx;
        }
        else return -1;
    }

    public double getTy() // degrees up or down relative to target
    {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            return ty;
        }
        else return -1;
    }

    public double getTa() // how big target is (0%->100%)
    {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ta = result.getTa();
            return ta;
        }
        else return -1;
    }

    public void updateTelemetry()
    {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // degrees left or right of target
            double ty = result.getTy(); // degrees up or down relative to target
            double ta = result.getTa(); // how big target is (0%->100% of img)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}