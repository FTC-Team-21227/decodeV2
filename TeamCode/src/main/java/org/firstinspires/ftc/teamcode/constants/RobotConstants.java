package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.Robot2;

public class RobotConstants {
    public static double P = 0.65; // Fraction of time along trajectory from ground to ground
    public static double G = 386.22; // Gravity (in/s^2)
    public final static double DELTA_H = Robot2.Positions.deltaH; // Height difference from shooter to goal
    public final static double FLIGHT_TIME = Math.sqrt(2 * DELTA_H / (P * G * (1 - P))); // Ball trajectory time from ground to ground
    public static double FLYWHEEL_MIN_VEL;
    public final static double FLYWHEEL_RADIUS = 1.89;
    public static double FLYWHEEL_POWER = 2.32;
    public final static double FLYWHEEL_TICKS_PER_REV = 28;
}
