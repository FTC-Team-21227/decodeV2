package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.constants.Constants;


public class Shooter {
    private Hood hood;
    private Turret turret;
    private Flywheel flywheel;
    private Intake2 intakeTransfer;
    private static ShooterSystem shooterSystem;

    // Constructor
    public Shooter(HardwareMap hardwareMap) {
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intakeTransfer = new Intake2(hardwareMap);
    }

    // CONSTANTS
    public double P = Constants.P; // Fraction of time along trajectory from ground to ground
    final double DELTA_H = Constants.DELTA_H; // Height difference from shooter to goal
    final double FLIGHT_TIME = Constants.FLIGHT_TIME; // Ball trajectory time from ground to ground
    double FLYWHEEL_MIN_VEL = Constants.FLYWHEEL_MIN_VEL;
    final double FLYWHEEL_RADIUS = Constants.FLYWHEEL_RADIUS;
    double FLYWHEEL_POWER = Constants.FLYWHEEL_POWER;
    final double FLYWHEEL_TICKS_PER_REV = Constants.FLYWHEEL_TICKS_PER_REV;

    // METHODS
    public Vector getGoalVector(Pose robotPose) {
        Pose turretPose = robotPose.plus(Constants.turretPos);
        return Constants.goalPos.minus(turretPose.getAsVector());
    }

    public double calculateFlywheelVel(Pose robotPose) {
        double goalDistance = getGoalVector(robotPose).getMagnitude();
        double theta = Math.atan(DELTA_H / (goalDistance * (1 - P))); // Ball launch angle of elevation
        double launchVel = goalDistance / (P * FLIGHT_TIME * Math.cos(theta));
        double radps = launchVel / FLYWHEEL_RADIUS;
        // Return flywheel velocity
        return radps * FLYWHEEL_POWER * FLYWHEEL_TICKS_PER_REV / (Math.PI * 2);
    }

    public double calculateHoodAngle(Pose robotPose) {
        double goalDistance = getGoalVector(robotPose).getMagnitude();
        // Calculate angle in radians
        double theta = Math.atan(DELTA_H / (goalDistance * (1 - P))); // Ball launch angle of elevation
        // Add offsets
        return theta + Constants.HOOD_OFFSET;

    }

    public double calculateTurretAngle(Pose robotPose) {
        double goalVectorAngle = getGoalVector(robotPose).getTheta();
        // Calculate angle
        double turretAngle = goalVectorAngle - robotPose.getHeading();
        // Add offsets
        return turretAngle + Constants.TURRET_OFFSET;
    }

    // Adjust the robot Pose based on its current movement
    public Pose adjustMovementShot(Pose robotPose, Vector linearVel, double angularVel) {
        double newHeading = robotPose.getHeading() + angularVel * FLIGHT_TIME;
        Pose correctedPose = new Pose(
                robotPose.getX() + linearVel.getXComponent() * FLIGHT_TIME,
                robotPose.getY() + linearVel.getYComponent() * FLIGHT_TIME,
                newHeading);
//        Pose correctedPose = new Pose(robotPose.getAsVector().plus(linearVel.times(FLIGHT_TIME)).getXComponent(),robotPose.getAsVector().plus(linearVel.times(FLIGHT_TIME)).getYComponent(),robotPose.getHeadingAsUnitVector().times(angularVel * FLIGHT_TIME).getMagnitude());
        return correctedPose;
    }

    // Shoots a specific number of times in auton
    public void autonShoot(Pose robotPose, int shotRequestCount) {
        double flywheelVel = calculateFlywheelVel(robotPose);
        double turretAngle = calculateTurretAngle(robotPose);
        double hoodAngle = calculateHoodAngle(robotPose);
        for (int i = 0; i < shotRequestCount; i++) {
            shootSequence(flywheelVel, hoodAngle, turretAngle);
        }
    }

    // Teleop shooting that allows human adjustment and human feeding
    public void updateTeleShots(boolean shoot, boolean humanFeed, boolean toggleLock, Pose robotPose, boolean flywheelUp, boolean flywheelDown, boolean hoodUp, boolean hoodDown, boolean turretLeft, boolean turretRight) {
        boolean shooterLocked = false;
        double flywheelVel = calculateFlywheelVel(robotPose);
        double hoodAngle = calculateHoodAngle(robotPose);
        double turretAngle = calculateTurretAngle(robotPose);

        if (toggleLock) {shooterLocked = !shooterLocked;}

        if (humanFeed) {
            flywheel.spinTo(Constants.HUMAN_FEED_VEL);
            hood.turnToAngle(Math.PI/2);
            turret.turnToRobotAngle(0);
        }
        else {
            if (shooterLocked) { // Locked
                // Adjustment values
                double adjustFlywheel = 0;
                double adjustHood = 0;
                double adjustTurret = 0;
                if(flywheelUp) {adjustFlywheel += 5;}
                if(flywheelDown) {adjustFlywheel -= 5;}
                if(hoodUp) {adjustHood += 5;}
                if(hoodDown) {adjustFlywheel -= 5;}
                if(turretLeft) {adjustTurret += 5;}
                if(turretRight) {adjustTurret -= 5;}

                flywheel.spinTo(flywheelVel + adjustFlywheel);
                hood.turnToAngle(hoodAngle + adjustHood);
                turret.turnToRobotAngle(turretAngle + adjustTurret);
                // Shoot
                if(shoot) {shootSequence(flywheelVel + adjustFlywheel, hoodAngle + adjustHood, turretAngle + adjustTurret);}
            }
            else { // Normal
                flywheel.spinTo(flywheelVel);
                hood.turnToAngle(hoodAngle);
                turret.turnToRobotAngle(turretAngle);
                // Shoot
                if(shoot) {shootSequence(flywheelVel, hoodAngle, turretAngle);}
            }
        }
    }

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        SHOOTING,
    } private LaunchState launchState;

    // Shooting sequence for one shot
    public void shootSequence(double flywheelVel, double hoodAngle, double turretAngle) {
        ElapsedTime spinUpTimer = new ElapsedTime(); // Time flywheel acceleration
        switch (launchState) {
            case IDLE:
                spinUpTimer.reset();
                flywheel.spinTo(flywheelVel);
                hood.turnToAngle(hoodAngle);
                turret.turnToRobotAngle(turretAngle);
                intakeTransfer.nextArtifact(); // Push balls to new spots
                launchState = LaunchState.SPIN_UP;
                break;
            case SPIN_UP: // Check if flywheel velocity is enough to shoot
                if(flywheel.getVel() > flywheelVel - 50 || spinUpTimer.seconds() > Robot.Constants.spinUpTimeout) {
                    launchState = LaunchState.SHOOTING;
                }
                break;
            case SHOOTING:
                intakeTransfer.shootArtifact(); // Move ball to the flywheel
                break;
        }
    }
}
