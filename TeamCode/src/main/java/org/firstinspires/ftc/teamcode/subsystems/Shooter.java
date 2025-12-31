package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;


public class Shooter {
    // SUBSYSTEMS AND TIMERS
    private Hood hood;
    private Turret turret;
    private Flywheel flywheel;
    private Feeder feeder;
    private Intake2 intake;
    Pose goalPose;

    int alreadyShot = 0;
    int shotRequests = 0;

    // CONSTANTS
    public double P = RobotConstants.P; // Fraction of time along trajectory from ground to ground
    final double G = RobotConstants.G; // Gravity (in/s^2)
    final double DELTA_H = RobotConstants.DELTA_H; // Height difference from shooter to goal
    final double FLIGHT_TIME = RobotConstants.FLIGHT_TIME; // Ball trajectory time from ground to ground
    double FLYWHEEL_MIN_VEL = RobotConstants.FLYWHEEL_MIN_VEL;
    final double FLYWHEEL_RADIUS = RobotConstants.FLYWHEEL_RADIUS;
    double FLYWHEEL_POWER = RobotConstants.FLYWHEEL_POWER;
    final double FLYWHEEL_TICKS_PER_REV = RobotConstants.FLYWHEEL_TICKS_PER_REV;

    // METHODS
    public Vector getGoalVector(Pose robotPose) {
        Pose turretPose = robotPose.plus(Robot2.Constants.turretPos);
        return Robot2.Positions.goalPos.minus(turretPose.getAsVector());
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
        return theta + Robot2.Constants.hoodAngleOffset;

    }

    public double calculateTurretAngle(Pose robotPose) {
        double goalVectorAngle = getGoalVector(robotPose).getTheta();
        // Calculate angle
        double turretAngle = goalVectorAngle - robotPose.getHeading();
        // Add offsets
        return turretAngle + Robot2.Constants.turretAngleOffset;
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

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        FEED_FRONT,
        FEED_BACK,
        PUSH_NEW_BALL,
        LAUNCHING,
        FEED_DOWN,
    } private LaunchState launchState;

    public void setShotRequests(int num) {
        this.shotRequests = num;
    }

    public int getShotRequests() {
        return this.shotRequests;
    }

    public void updateAutonShots(Pose robotPose) {
        // TIMERS and VARIABLES
        ElapsedTime feedTimer= new ElapsedTime(); // Time feeding
        ElapsedTime spinUpTimer = new ElapsedTime(); // Time flywheel acceleration
        boolean shootRightSide = true;
        double flywheelVel = calculateFlywheelVel(robotPose);
        double turretAngle = calculateTurretAngle(robotPose);
        double hoodAngle = calculateHoodAngle(robotPose);
        // SEQUENCE

        /**
         * IDLE:
         * set feeders down
         * set shooter stuff (flywheel vel, hood and turret angles)
         * if one already shot, small intake pulse, normal stuff
         * SPIN UP:
         * check vel and change state
         * feeder timer reset
         * check side to decide whether to go to feed right or left
         * FEED__:
         * stop intake
         * feeder up
         * feeder timer reset
         * LAUNCHING:
         * check if feed had enough time to go up
         * feeders go down
         * alternate feed side, alreadyshot++
         * FEED DOWN:
         * have enough time to go down
         * intake can continue
         */
        switch (launchState) {
            case IDLE:
                spinUpTimer.reset();
                feeder.downBL();
                feeder.downFR();
                flywheel.spinTo(flywheelVel);
                hood.turnToAngle(hoodAngle);
                turret.turnToRobotAngle(turretAngle);
                if (alreadyShot > 1) { // Push balls to new spots
                    intake.smallIntake();
                }
                launchState = LaunchState.SPIN_UP;
                break;
//            case SPIN_UP:
//                if(flywheel.getVel() > flywheel.getTargetVel() - 50 || spinUpTimer.seconds() > Robot2.Constants.spinUpTimeout) {
//                    launchState = LaunchState.FEED_FRONT
//                }
//                break;
//            case FEED_FRONT:
//                intake.pause();
//                feeder.upFR(); // feeder starts
//                feederTimer.reset(); // feeder goes down
//                launchState = Robot2.LaunchState.LAUNCHING;
//                break;
//            case FEED_BACK:
//                intake.pause();
//                feeder.upBL();
//                feederTimer.reset();
//                launchState = Robot2.LaunchState.LAUNCHING;
//                break;
//            case LAUNCHING:
//                if (feederTimer.seconds() > Robot2.Constants.feedTime) {
//                    launchState = Robot2.LaunchState.FEED_DOWN;
//                    feeder.downFR();
//                    feeder.downBL();
//                    feederTimer.reset();
//                    shotReqFeederType = !shotReqFeederType;
//                }
//                break;
//            case FEED_DOWN:
//                if (feederTimer.seconds() > Robot2.Constants.feedDownTime) {
//                    launchState = Robot2.LaunchState.IDLE;
//                    feederTimer.reset();
//                    intake.proceed();
//                    if (shotReqAlt) chainShotCount++;
//                }
//                break;
        } // --------------------END OF STATE MANAGER--------------------------------
    }

    // TODO: add manual adjustment method
    // TODO: add human feed method
}
