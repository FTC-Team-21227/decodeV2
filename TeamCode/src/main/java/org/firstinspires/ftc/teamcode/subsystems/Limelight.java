package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


public class Limelight {
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;

    public Limelight(HardwareMap hardwareMap) // initialization constructor
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // currently tuned pipeline
        limelight.setPollRateHz(100); // update data 100 times/s
        limelight.start(); // starts limelight
    }

    // Returns camera's field-relative position
    public Pose update(double headingDegrees /*degrees*/, Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        Pose2D pose2d = pinpoint.getPosition();
        double robotYaw = pose2d.getHeading(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw + 180); // update for megatag 2

//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        limelight.updateRobotOrientation(headingDegrees + 180);
//        Pose2D pose2d = pinpoint.getPosition();
//        double robotYaw = pose2d.getHeading(AngleUnit.DEGREES);
//        limelight.updateRobotOrientation(robotYaw + 180); // update for megatag 2
//
////        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
////        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
//
//=======
//>>>>>>> c3e49e188f8fd4005824735df0a0a594ff98d47f
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // horizontal offset (deg)
            double ty = result.getTy(); // vertical offset (deg)

            Pose3D botpose = result.getBotpose_MT2();
            // pose3d, field relative, coords + orientation

            double x = botpose.getPosition().x; // x
            double y = botpose.getPosition().y; // y
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
    public double getDistanceToGoal() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        final double cameraHeightInches = 8.0;      // inches
        final double goalHeightInches = 56.0;
        double limelightMountAngleDegrees = 0;

        double ty = result.getTy(); // vertical offset in degrees
        double angleToGoalDegrees = limelightMountAngleDegrees + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        double distanceFromLimelightToGoalInches = (goalHeightInches - cameraHeightInches)/Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }
}