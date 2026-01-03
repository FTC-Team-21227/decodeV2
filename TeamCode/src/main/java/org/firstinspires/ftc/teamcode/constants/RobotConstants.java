package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class RobotConstants {
    public static double P = 0.65; // Fraction of time along trajectory from ground to ground
    public static double G = 386.22; // Gravity (in/s^2)
    public final static double DELTA_H = 39; // Height difference from shooter to goal
    public final static double DELTA_H_FAR = 41; // Height difference from shooter to goal (shoot higher)
    public final static double FLIGHT_TIME = Math.sqrt(2 * DELTA_H / (P * G * (1 - P))); // Ball trajectory time from ground to ground
    public static double FLYWHEEL_MIN_VEL;
    public final static double FLYWHEEL_RADIUS = 1.89;
    public static double FLYWHEEL_POWER = 2.32;
    public final static double FLYWHEEL_TICKS_PER_REV = 28;
    public static double HUMAN_FEED_VEL = -676767;
    public static double HOOD_OFFSET = 0;

    public final static boolean MINIMIZE_TELEMETRY = false;

    // GOAL POSES
    public final static Vector goalPos = new Vector(Math.hypot(-58.3727 - 5.3, 55.6425 + 5.3), Math.atan2(55.6425 + 5.3, -58.3727 - 5.3)); //distance to triangle center = 7.5 inch
    public final static Vector goalPosFar = new Vector(Math.hypot(-70, 55.6425 + 12), Math.atan2(55.6425 + 12, -70)); //distance to triangle center = 7.5 inch
    // SHOT POSES
    public final static Pose autoShotPose = new Pose(-43,33, Math.toRadians(90)); //new Pose2d(-12,15,Math.toRadians(90));
    public final static Pose autoShotPoseFar = new Pose(56,12, Math.toRadians(120));
    public final static Pose teleShotPose = new Pose(0,0,goalPos.getTheta() + Math.PI);
    public final static Pose teleShotPoseFar = new Pose(0,0,goalPos.getTheta() + Math.PI);
    // FLYWHEEL
    public final static double spinUpTimeout = 2;
    public final static double kP = 0.052, kI = 0, kD = 0.000, kF = 10 /*kF will be removed in our new version*/, kS = 0.65, kV = 0.00450;
    // HOOD
    public final static double hoodLowAngle = 65.36742754 * Math.PI/180; // the traj angle from horizonatla (rad) //75 //0;
    public final static double hoodHighAngle = 24.47717215 * Math.PI/180; //30 //50*Math.PI/180; //the traj angle from horizontal 55; // Highest actual degree is 41
    public final static double hoodScale0 = 0.2; //0.27;
    public final static double hoodScale1 = 0.8; //1; //0.85;
    // TURRET
    public static double TURRET_OFFSET = 0;
    public final static Pose turretPos = new Pose(-1.512,-0.12224409,0);
    public final static double TURRET_HIGH_ANGLE = 3*Math.PI/2; //164.7*Math.PI/180; //220*Math.PI/180;; //355*Math.PI/180; // //140*Math.PI/180; // In rad, pos = 1
    public final static double TURRET_LOW_ANGLE = Math.PI/2; //-175*Math.PI/180; //-40*Math.PI/180;; // //-208*Math.PI/180; // In rad (= old -330 deg)
    //        public final static double turretTargetRangeOffset = TURRET_HIGH_ANGLE-Math.PI; //offset from (-pi,pi)
    // Offset from (-pi,pi) to (midpoint-pi, midpoint+pi), i.e. shift midpoint from 0 to new midpoint
    public final static double turretTargetRangeOffset = Math.PI/2; //(TURRET_LOW_ANGLE + TURRET_HIGH_ANGLE )/2.0; //TURRET_HIGH_ANGLE-Math.PI;
    public final static double TURRET_SCALE_0 = 0.33055555555555555; //0.218; //0; //0.25 ;//0; //0.11;
    public final static double TURRET_SCALE_1 = 0.7838888888888889; //0.67; //1; //0.78; //0.86; //1;
    // DRIVETRAIN
    public final static double DRIVE_POWER = 1.0;
    public final static double DRIVE_POWER_TELE = 1.0;
    public final static double DRIVE_POWER_SLOW = 0.2;

}
