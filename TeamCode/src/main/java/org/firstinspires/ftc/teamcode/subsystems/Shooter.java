package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterSystem.createDefaultTable;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


public class Shooter {
    private Hood hood;
    private Turret turret;
    public Flywheel flywheel;
    public ShooterSystem.ShooterLookupTable table;

    // CONSTANTS
    public double P = 0.65; // Fraction of time along trajectory from ground to ground
    public double G = 386.22;
    final double DELTA_H = Robot.Constants.deltaH; // Height difference from shooter to goal
//    final double FLIGHT_TIME = Robot.Constants.; // Ball trajectory time from ground to ground flight time isn't a constant
//    double FLYWHEEL_MIN_VEL = Robot.Constants.FLYWHEEL_MIN_VEL;
//    final double FLYWHEEL_RADIUS = Robot.Constants.FLYWHEEL_RADIUS;
//    final double FLYWHEEL_TICKS_PER_REV = Robot.Constants.FLYWHEEL_TICKS_PER_REV;

    double flywheelVel;
    double hoodAngle;
    double turretAngle;

    public Shooter (HardwareMap hardwareMap){
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        table = createDefaultTable();
    }
    // METHODS
    public Vector getGoalVector(Pose robotPose) {
        Pose turretPose = robotPose.plus(Robot.Constants.turretPos);
        return Robot.Constants.goalPos.minus(turretPose.getAsVector());
    }

//    public double calculateFlywheelVel(Pose robotPose) {
//        double goalDistance = getGoalVector(robotPose).getMagnitude();
//        double theta = Math.atan(DELTA_H / (goalDistance * (1 - P))); // Ball launch angle of elevation
//        double flightTime = Math.sqrt(2 * DELTA_H / (p * g * (1 - p))); // Ball trajectory time from ground to ground
//
//        double launchVel = goalDistance / (P * FLIGHT_TIME * Math.cos(theta));
//        double radps = launchVel / FLYWHEEL_RADIUS;
//        // Return flywheel velocity
//        return radps * (Robot.Positions.flywheelPower + Robot.Positions.flywheelPowerOffset) * FLYWHEEL_TICKS_PER_REV / (Math.PI * 2);
//    }
//
//    public double calculateHoodAngle(Pose robotPose) {
//        double goalDistance = getGoalVector(robotPose).getMagnitude();
//        // Calculate angle in radians
//        double theta = Math.atan(DELTA_H / (goalDistance * (1 - P))); // Ball launch angle of elevation
//        // Add offsets
//        return theta + Robot.Constants.hoodAngleOffset + Robot.Positions.hoodAngleManualOffset;
//
//    }
//
//    public double calculateTurretAngle(Pose robotPose) {
//        double goalVectorAngle = getGoalVector(robotPose).getTheta();
//        // Calculate angle
//        double turretAngle = goalVectorAngle - robotPose.getHeading();
//        // Add offsets
//        return turretAngle + Robot.Constants.turretAngleOffset + Robot.Positions.turretAngleManualOffset;
//    }

    // Adjust the robot Pose based on its current movement
    public Pose adjustMovementPose(Pose robotPose, Vector linearVel, double angularVel) {
        double p = 0.65; // Fraction of time along trajectory from ground to ground
        double g = 386.22; // Gravity (in/s^2)
        double flightTime = Math.sqrt(2 * Robot.Positions.deltaH / (p * g * (1 - p))); // Ball trajectory time from ground to ground
        double newHeading = robotPose.getHeading() + angularVel * flightTime; //actually ang velocity shouldn't affect the estimated shot because the turret is near the center enough
        Pose correctedPose = new Pose(
                robotPose.getX() + linearVel.getXComponent() * flightTime,
                robotPose.getY() + linearVel.getYComponent() * flightTime,
                robotPose.getHeading() /*newHeading*/);
//        Pose correctedPose = new Pose(robotPose.getAsVector().plus(linearVel.times(FLIGHT_TIME)).getXComponent(),robotPose.getAsVector().plus(linearVel.times(FLIGHT_TIME)).getYComponent(),robotPose.getHeadingAsUnitVector().times(angularVel * FLIGHT_TIME).getMagnitude());
        return correctedPose;
    }

    public boolean isAimed(){
        return Math.abs(flywheel.getVel() - flywheelVel) < 50 && Math.abs(turret.getTurretRobotAngle() - turretAngle) < 0.1; //incl servo angles later IF we use axon
    }
//    // Shoots a specific number of times in auton
//    public void autonShoot(Pose robotPose, int shotRequestCount) {
//        double flywheelVel = calculateFlywheelVel(robotPose);
//        double turretAngle = calculateTurretAngle(robotPose);
//        double hoodAngle = calculateHoodAngle(robotPose);
//        for (int i = 0; i < shotRequestCount; i++) {
//            shootSequence(flywheelVel, hoodAngle, turretAngle);
//        }
//    }

    //shooting that allows human adjustment and human feeding
    public void update(boolean shoot, boolean humanFeed, Pose robotPose, boolean flywheelUp, boolean flywheelDown, boolean hoodUp, boolean hoodDown, boolean turretLeft, boolean turretRight) {
        // Adjustment values, static so they can be easily accessed
        if(flywheelUp) {Robot.Positions.flywheelPowerOffset += 5;}
        if(flywheelDown) {Robot.Positions.flywheelPowerOffset -= 5;}
        if(hoodUp) {Robot.Positions.hoodAngleManualOffset += 5;}
        if(hoodDown) {Robot.Positions.hoodAngleManualOffset -= 5;}
        if(turretLeft) {Robot.Positions.turretAngleManualOffset += 5;}
        if(turretRight) {Robot.Positions.turretAngleManualOffset -= 5;}

        // ------------------------Shooting trajectory values-----------------------
        double p = 0.65; // Fraction of time along trajectory from ground to ground
        double g = 386.22; // Gravity (in/s^2)
        double deltaH = Robot.Positions.deltaH; // Height difference from shooter to goal
        double flightTime = Math.sqrt(2 * deltaH / (p * g * (1 - p))); // Ball trajectory time from ground to ground

        // Calculate shot vector using TURRET's position on the robot and ROBOT's heading---
        Pose pose = robotPose.plus(Robot.Constants.turretPos); // Turret's field-relative position
        Vector goalVector = Robot.Positions.goalPos.minus(pose.getAsVector());
        double distance = goalVector.getMagnitude(); // Horizontal distance
        double goalVectorAngle = goalVector.getTheta(); // Angle

        // -----Calculate hood angle, flywheel velocity, and turret angle--------
        double theta = Math.atan(deltaH / (distance * (1 - p))); // Ball launch angle of elevation
        double vel = distance / (p * flightTime * Math.cos(theta)); // Ball launch speed
        double heading = pose.getHeading(); //
        turretAngle = goalVectorAngle - heading; // Angle to turn turret to (relative to robot's heading)
        double wheelRadius = 1.89;

        // Convert vel (speed) to rad/s (example calibration: vel = wheelRadius * rad/s
        double radps = vel / wheelRadius; // RPM

        ShooterSystem.ShooterPoint point = table.get(distance, 14);
        flywheelVel = radps * 28 / Math.PI / 2 * (Robot.Positions.flywheelPower + Robot.Positions.flywheelPowerOffset);
//        flywheelVel = point.rpm;
        hoodAngle = theta+ Robot.Constants.hoodAngleOffset+ Robot.Positions.hoodAngleManualOffset;
//        hoodAngle = point.distance;
        turretAngle = turretAngle+ Robot.Constants.turretAngleOffset+ Robot.Positions.turretAngleManualOffset;


        if (!shoot){
            flywheel.setPower(0);
            //don't move hood and turret
        }
        else if (humanFeed) {
            flywheel.spinTo(Robot.Constants.humanFeedVel);
            hood.turnToAngle(Math.PI/2);
            turret.turnToRobotAngle(0);
        }
        else {
            flywheel.spinTo(flywheelVel);
            hood.turnToAngle(hoodAngle);
            turret.turnToRobotAngle(turretAngle);
        }
//        telemetry.update();
    }
    public void sendTelemetry(Telemetry telemetry){
        // ---------------------------TELEMETRY LINES--------------------------------
        if (hood.HOOD.commandedOutsideRange()) telemetry.addLine("WARNING: hood commanded out of its range! Auto set to 0 or 1.");
//        if (turret.turret.commandedOutsideRange()) telemetry.addLine("WARNING: turret commanded out of its range! Auto set to 0 or 1.");
        telemetry.addData("flywheel power scale factor", Robot.Positions.flywheelPower + Robot.Positions.flywheelPowerOffset);
        if (!Robot.Constants.MINIMIZE_TELEMETRY) {
//            telemetry.addData("human feed", humanFeed); //NOT IMPORTANT
//            telemetry.addData("setPose", toggleLock); //NOT IMPORTANT
        }
//        telemetry.addLine("robotTurretPose (inchxinchxdeg): " + robotPose.getX()+" "+robotPose.getY()+" "+robotPose.getHeading()); //NOT IMPORTANT
//        telemetry.addLine("goalVector (inchxinch): " + goalVector.getXComponent()+" "+goalVector.getYComponent());
        telemetry.addLine("goalPos (inchxinch): " + Robot.Positions.goalPos.getXComponent()+" "+ Robot.Positions.goalPos.getYComponent()); //NOT IMPORTANT
        if (!Robot.Constants.MINIMIZE_TELEMETRY) {
//            telemetry.addData("goalVector angle (rad to deg)", Math.toDegrees(goalVectorAngle)); //NOT IMPORTANT
//            telemetry.addData("distance to goal (inch)", distance); //NOT IMPORTANT
        }
//        telemetry.addData("turret angle (rad to deg)", turretAngle*180/Math.PI);
        telemetry.addData("turret angle offset (rad to deg)", Robot.Constants.turretAngleOffset*180/Math.PI);
        telemetry.addData("turret angle offset manual(rad to deg)", Robot.Positions.turretAngleManualOffset*180/Math.PI);
        if (!Robot.Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("turret get angle (rad to deg)", turret.getTurretRobotAngle() * 180 / Math.PI); //NOT IMPORTANT
        }
        telemetry.addData("turret pos", turret.turret.getCurrentAngle()); //NOT IMPORTANT
//        telemetry.addData("hood theta (rad to deg)", theta*180/Math.PI);
        telemetry.addData("hood angle offset (rad to deg)", Robot.Constants.hoodAngleOffset*180/Math.PI);
        telemetry.addData("hood angle offset manual (rad to deg)", Robot.Positions.hoodAngleManualOffset*180/Math.PI);
        if (!Robot.Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("hood get angle (rad to deg)", hood.getAngle()); //NOT IMPORTANT
        }
        telemetry.addData("hood pos", hood.HOOD.getPosition()); //NOT IMPORTANT
//        telemetry.addData("targetVel (rad/s to tick/s)", radps*28/Math.PI/2*(Robot.Positions.flywheelPower+ Robot.Positions.flywheelPowerOffset));
        if (!Robot.Constants.MINIMIZE_TELEMETRY) {
//            telemetry.addData("targetVel (rad/s)", radps); //NOT IMPORTANT
        }
        telemetry.addData("motorSpeed (tick/s)", flywheel.getVel());
        if (!Robot.Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("motorSpeed (tick/s to rad/s)", flywheel.getVel() * 2 * Math.PI / 28); //NOT IMPORTANT
        }
    }
}
