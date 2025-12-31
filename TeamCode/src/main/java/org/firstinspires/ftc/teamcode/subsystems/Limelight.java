package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Limelight {
    Limelight3A limelight;

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

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // horizontal offset (deg)
            double ty = result.getTy(); // vertical offset (deg)

            // UPDATE CAM CONSTANTS-----------
            final double CAMERA_HEIGHT = 10.0; // camera height off ground
            final double TARGET_HEIGHT = 24.0; // height of goal/target
            final double CAMERA_PITCH = 0.0; // deg
            final double TARGET_X = 144.0; // x-coord of target
            final double TARGET_Y = 72.0;  // y-coord of target

            // Convert vertical angle to radians
            double angleToTargetRad = Math.toRadians(CAMERA_PITCH + ty);

            // Distance from camera to target
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToTargetRad);

            // robot heading relative to target
            double robotHeadingToTargetRad = Math.toRadians(tx);

            // field-relative robot pos
            double robotX = TARGET_X - distance * Math.cos(robotHeadingToTargetRad);
            double robotY = TARGET_Y - distance * Math.sin(robotHeadingToTargetRad);

            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Distance to Target", distance);

            return new Pose(robotX, robotY, tx + 45); // field heading based on target heading??
            // UPDATE HEADING LATER
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