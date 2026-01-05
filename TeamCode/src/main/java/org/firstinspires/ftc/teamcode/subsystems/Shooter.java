package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.constants.RobotPositions;


public class Shooter {
    private Hood hood;
    private Turret turret;
    public Flywheel flywheel;

    // CONSTANTS
    public double P = RobotConstants.P; // Fraction of time along trajectory from ground to ground
    final double DELTA_H = RobotConstants.DELTA_H; // Height difference from shooter to goal
    final double FLIGHT_TIME = RobotConstants.FLIGHT_TIME; // Ball trajectory time from ground to ground
    double FLYWHEEL_MIN_VEL = RobotConstants.FLYWHEEL_MIN_VEL;
    final double FLYWHEEL_RADIUS = RobotConstants.FLYWHEEL_RADIUS;
    final double FLYWHEEL_TICKS_PER_REV = RobotConstants.FLYWHEEL_TICKS_PER_REV;

    double flywheelVel;
    double hoodAngle;
    double turretAngle;

    public Shooter (HardwareMap hardwareMap){
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
    }
    // METHODS
    public Vector getGoalVector(Pose robotPose) {
        Pose turretPose = robotPose.plus(RobotConstants.turretPos);
        return RobotConstants.goalPos.minus(turretPose.getAsVector());
    }

    public double calculateFlywheelVel(Pose robotPose) {
        double goalDistance = getGoalVector(robotPose).getMagnitude();
        double theta = Math.atan(DELTA_H / (goalDistance * (1 - P))); // Ball launch angle of elevation
        double launchVel = goalDistance / (P * FLIGHT_TIME * Math.cos(theta));
        double radps = launchVel / FLYWHEEL_RADIUS;
        // Return flywheel velocity
        return radps * (RobotPositions.flywheelPower + RobotPositions.flywheelPowerOffset) * FLYWHEEL_TICKS_PER_REV / (Math.PI * 2);
    }

    public double calculateHoodAngle(Pose robotPose) {
        double goalDistance = getGoalVector(robotPose).getMagnitude();
        // Calculate angle in radians
        double theta = Math.atan(DELTA_H / (goalDistance * (1 - P))); // Ball launch angle of elevation
        // Add offsets
        return theta + RobotConstants.HOOD_OFFSET + RobotPositions.hoodAngleManualOffset;

    }

    public double calculateTurretAngle(Pose robotPose) {
        double goalVectorAngle = getGoalVector(robotPose).getTheta();
        // Calculate angle
        double turretAngle = goalVectorAngle - robotPose.getHeading();
        // Add offsets
        return turretAngle + RobotConstants.TURRET_OFFSET + RobotPositions.turretAngleManualOffset;
    }

    // Adjust the robot Pose based on its current movement
    public Pose adjustMovementPose(Pose robotPose, Vector linearVel, double angularVel) {
        double newHeading = robotPose.getHeading() + angularVel * FLIGHT_TIME; //actually ang velocity shouldn't affect the estimated shot because the turret is near the center enough
        Pose correctedPose = new Pose(
                robotPose.getX() + linearVel.getXComponent() * FLIGHT_TIME,
                robotPose.getY() + linearVel.getYComponent() * FLIGHT_TIME,
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
        if(flywheelUp) {RobotPositions.flywheelPowerOffset += 5;}
        if(flywheelDown) {RobotPositions.flywheelPowerOffset -= 5;}
        if(hoodUp) {RobotPositions.hoodAngleManualOffset += 5;}
        if(hoodDown) {RobotPositions.hoodAngleManualOffset -= 5;}
        if(turretLeft) {RobotPositions.turretAngleManualOffset += 5;}
        if(turretRight) {RobotPositions.turretAngleManualOffset -= 5;}

        flywheelVel = calculateFlywheelVel(robotPose);
        hoodAngle = calculateHoodAngle(robotPose);
        turretAngle = calculateTurretAngle(robotPose);

        if (!shoot){
            flywheel.setPower(0);
            //don't move hood and turret
        }
        else if (humanFeed) {
            flywheel.spinTo(RobotConstants.HUMAN_FEED_VEL);
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
        telemetry.addData("flywheel power scale factor", RobotPositions.flywheelPower + RobotPositions.flywheelPowerOffset);
        if (!RobotConstants.MINIMIZE_TELEMETRY) {
//            telemetry.addData("human feed", humanFeed); //NOT IMPORTANT
//            telemetry.addData("setPose", toggleLock); //NOT IMPORTANT
        }
//        telemetry.addLine("robotTurretPose (inchxinchxdeg): " + robotPose.getX()+" "+robotPose.getY()+" "+robotPose.getHeading()); //NOT IMPORTANT
//        telemetry.addLine("goalVector (inchxinch): " + goalVector.getXComponent()+" "+goalVector.getYComponent());
        telemetry.addLine("goalPos (inchxinch): " + RobotPositions.goalPos.getXComponent()+" "+ RobotPositions.goalPos.getYComponent()); //NOT IMPORTANT
        if (!RobotConstants.MINIMIZE_TELEMETRY) {
//            telemetry.addData("goalVector angle (rad to deg)", Math.toDegrees(goalVectorAngle)); //NOT IMPORTANT
//            telemetry.addData("distance to goal (inch)", distance); //NOT IMPORTANT
        }
//        telemetry.addData("turret angle (rad to deg)", turretAngle*180/Math.PI);
        telemetry.addData("turret angle offset (rad to deg)", RobotConstants.TURRET_OFFSET*180/Math.PI);
        telemetry.addData("turret angle offset manual(rad to deg)", RobotPositions.turretAngleManualOffset*180/Math.PI);
        if (!RobotConstants.MINIMIZE_TELEMETRY) {
            telemetry.addData("turret get angle (rad to deg)", turret.getTurretRobotAngle() * 180 / Math.PI); //NOT IMPORTANT
        }
        telemetry.addData("turret pos", turret.turret.getCurrentAngle()); //NOT IMPORTANT
//        telemetry.addData("hood theta (rad to deg)", theta*180/Math.PI);
        telemetry.addData("hood angle offset (rad to deg)", RobotConstants.HOOD_OFFSET*180/Math.PI);
        telemetry.addData("hood angle offset manual (rad to deg)", RobotPositions.hoodAngleManualOffset*180/Math.PI);
        if (!RobotConstants.MINIMIZE_TELEMETRY) {
            telemetry.addData("hood get angle (rad to deg)", hood.getAngle()); //NOT IMPORTANT
        }
        telemetry.addData("hood pos", hood.HOOD.getPosition()); //NOT IMPORTANT
//        telemetry.addData("targetVel (rad/s to tick/s)", radps*28/Math.PI/2*(RobotPositions.flywheelPower+ RobotPositions.flywheelPowerOffset));
        if (!RobotConstants.MINIMIZE_TELEMETRY) {
//            telemetry.addData("targetVel (rad/s)", radps); //NOT IMPORTANT
        }
        telemetry.addData("motorSpeed (tick/s)", flywheel.getVel());
        if (!RobotConstants.MINIMIZE_TELEMETRY) {
            telemetry.addData("motorSpeed (tick/s to rad/s)", flywheel.getVel() * 2 * Math.PI / 28); //NOT IMPORTANT
        }
    }




}
