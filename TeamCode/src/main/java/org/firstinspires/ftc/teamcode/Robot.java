package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.pedropathing.Drivetrain;
import com.pedropathing.ErrorCalculator;
import com.pedropathing.VectorCalculator;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.BallDetector;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


// FULL ROBOT CLASS: drive, flywheel, turret, hood, feeder, intake, camera
public class Robot {
    private static Robot instance = null;

    // --------------SUBSYSTEMS---------------------------------------------------------------------
    AprilFollower follower;
    Intake intake;
    Shooter shooter;
//    AprilTagLocalization2 camera; // Camera subsystem used in AprilDrive and Obelisk detection, switch to limelight later
    Limelight camera;
    BallDetector ballDetector;
//    Voltage voltageSensor;

    // ---------------BOOLEANS AND ATTRIBUTES-------------------------------------------------------
    boolean shotReqFeederType = true; // True = front feeder
    boolean lockStarted = false; // Lock shooting values to allow manual adjustment
    boolean tapping = false;
    double turretAngle;
    double radps; // Flywheel speed
    double theta; // Hood angle
    double chainShotCount = 1; // How many balls to shoot consecutively
    double lastTime = 0; // Previous time, used to calculate loop time
    int driveSideSign = -1; // Alliance color changes forward vs backwards direction
    Pose txWorldPinpoint = new Pose(0,0,Math.PI); // Robot pose based on pinpoint in inverted FTC coordinates (not pedro coordinates)
    Vector robotVel = new Vector(0,0);
    double robotAngVel = 0;


    // ---------------------------TIMERS------------------------------------------------------------
    ElapsedTime feederTimer;
    ElapsedTime aprilTimer; //gonna use aprilTimer to also track loop times. Sorry, not sorry.
    ElapsedTime intakeTimer; //this is so cancer

    // --------------------------- ROBOT ENUMS------------------------------------------------------
    // Enum that manages shooter state machine
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        SHOOTING,
    } private LaunchState launchState; // Instance

    //enum that manages intake state
    private enum IntakeState {
        IDLE,
        INTAKING,
        OUTTAKING,
        TRANSFERRING
    } private IntakeState intakeState; // Instance


    // Enum that switches between pinpoint and camera localization
    private enum DriveState {
        RELATIVE,
        ABSOLUTE, // Drive with AprilTag localization for absolute position on field
        ABSOLUTE2
    } private DriveState driveState; // Instance

    // Enum that stores the alliance color, accessible globally
    public enum Color {
        RED,
        BLUE
    } public Color color; // Instance

    // Enum that stores the opmode type, accessible globally
    public enum OpModeState {
        AUTO,
        AUTO_FAR,
        TELEOP,
        TELEOP_FAR
    } public OpModeState opModeState; // Instance

    // -------------------PUBLIC ROBOT CONSTRUCTOR--------------------------------------------------
    /**
     * Creates a robot instance with field-relative position starting at initialPose, and mirrors
     * shooting position and goal positions based on alliance color
     * @param initialPose
     * @param color
     */
    public Robot(Pose initialPose /*in pedro coords*/, Color color){
        // Poses are mirrored if BLUE
        this.color = color;
        if (this.color== Color.RED) {
            driveSideSign = -1;
            Positions.goalPos = RobotConstants.goalPos;
            // Where the robot will shoot from:
            Positions.autoShotPose = RobotConstants.autoShotPose;
            Positions.autoShotPose_Far = RobotConstants.autoShotPoseFar;
            Positions.teleShotPose = RobotConstants.teleShotPose;
            Positions.deltaH = RobotConstants.DELTA_H;
            RobotLog.d("It's Red");
        }
        else if (this.color == Color.BLUE){
            driveSideSign = 1;
            Positions.goalPos = mirrorVector(RobotConstants.goalPos);
            // Where the robot will shoot from:
            Positions.autoShotPose = RobotConstants.autoShotPose.mirror();
            Positions.autoShotPose_Far = RobotConstants.autoShotPoseFar.mirror();
            Positions.teleShotPose = RobotConstants.teleShotPose.mirror();
            Positions.deltaH = RobotConstants.DELTA_H;
            RobotLog.d("It's Blue");
        }
        else {
            RobotLog.d("What...");
        }
        // Robot's field relative pose, which starts at initialPose
        txWorldPinpoint = initialPose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
//        voltageSensor = new Voltage(hardwareMap.get(VoltageSensor.class,"Control Hub"));
    }

    // --------------------SINGLETON CONSTRUCTOR AND GETTER-----------------------------------------
    /**
     * Create one instance of robot (singleton).
     * NOTE: ALL DEVICES MUST BE REINITIALIZED BEFORE EVERY OPMODE, THEY ARE NOT SAVED.
     */
    // Get singleton instance
    public static Robot getInstance(Pose initialPose, Color color){
        if (instance == null || instance.opModeState == OpModeState.TELEOP){
            RobotLog.d("making a new instance");
            instance = new Robot(initialPose, color);
        }
        else RobotLog.d("keeping the instance");
        return instance;
    }

    // Create new instance
    public static Robot startInstance(Pose initialPose, Color color){
        instance = new Robot(initialPose, color);
        return instance;
    }

    // Clear singleton instance
    public static void clearInstance(){
        instance = null;
    }


    // --------------------DRIVETRAIN CLASS OF ROBOT------------------------------------------------
    /**
     * Resets localizer using AprilTags
     * Extends Roadrunner MecanumDrive Class
     */
    public class AprilFollower{
        private final Follower base;

        public AprilFollower(Follower base) {
            this.base = base;
        }

        public void updateConstants() {
            base.updateConstants();
        }

        public void followPath(Path path, boolean holdEnd) {
            base.followPath(path, holdEnd);
        }

        public void startTeleOpDrive(boolean useBrakeMode) {
            base.startTeleOpDrive(useBrakeMode);
        }

        public void updatePose() {
            base.updatePose();
        }

        public Vector getCentripetalForceCorrection() {
            return base.getCentripetalForceCorrection();
        }

        public void activateAllPIDFs() {
            base.activateAllPIDFs();
        }

        public void setMaxPower(double set) {
            base.setMaxPower(set);
        }

        public void followPath(PathChain pathChain, boolean holdEnd) {
            base.followPath(pathChain, holdEnd);
        }

        public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
            return base.atPose(pose, xTolerance, yTolerance);
        }

        public double getHeadingGoal(double t) {
            return base.getHeadingGoal(t);
        }

        public Path getCurrentPath() {
            return base.getCurrentPath();
        }

        public String[] debug() {
            return base.debug();
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric, double offsetHeading) {
            base.setTeleOpDrive(forward, strafe, turn, isRobotCentric, offsetHeading);
        }

        public PathChain getCurrentPathChain() {
            return base.getCurrentPathChain();
        }

        public void setDrivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) {
            base.setDrivePIDFCoefficients(drivePIDFCoefficients);
        }

        public void activateCentripetal() {
            base.activateCentripetal();
        }

        public Vector getTeleopDriveVector() {
            return base.getTeleopDriveVector();
        }

        public VectorCalculator getVectorCalculator() {
            return base.getVectorCalculator();
        }

        public void followPath(PathChain pathChain) {
            base.followPath(pathChain);
        }

        public void updateCallbacks() {
            base.updateCallbacks();
        }

        public boolean isRobotStuck() {
            return base.isRobotStuck();
        }

        public Vector getDriveVector() {
            return base.getDriveVector();
        }

        public void holdPoint(BezierPoint point, double heading) {
            base.holdPoint(point, heading);
        }

        public void setTeleOpDrive(double forward, double strafe, double turn) {
            base.setTeleOpDrive(forward, strafe, turn);
        }

        public boolean getUseTranslational() {
            return base.getUseTranslational();
        }

        public void setSecondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) {
            base.setSecondaryHeadingPIDFCoefficients(secondaryHeadingPIDFCoefficients);
        }

        public void startTeleopDrive() {
            base.startTeleopDrive();
        }

        public boolean atParametricEnd() {
            return base.atParametricEnd();
        }

        public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
            return base.atPose(pose, xTolerance, yTolerance, headingTolerance);
        }

        public Vector getClosestPointTangentVector() {
            return base.getClosestPointTangentVector();
        }

        public int getChainIndex() {
            return base.getChainIndex();
        }

        public void updateDrivetrain() {
            base.updateDrivetrain();
        }

        public Pose getPointFromPath(double t) {
            return base.getPointFromPath(t);
        }

        public PathBuilder pathBuilder(PathConstraints constraints) {
            return base.pathBuilder(constraints);
        }

        public void setSecondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) {
            base.setSecondaryDrivePIDFCoefficients(secondaryDrivePIDFCoefficients);
        }

        public void updateErrorAndVectors() {
            base.updateErrorAndVectors();
        }

        public double getDistanceRemaining() {
            return base.getDistanceRemaining();
        }

        public void startTeleOpDrive() {
            base.startTeleOpDrive();
        }

        public double getTotalHeading() {
            return base.getTotalHeading();
        }

        public Vector getTranslationalCorrection() {
            return base.getTranslationalCorrection();
        }

        public ErrorCalculator getErrorCalculator() {
            return base.getErrorCalculator();
        }

        public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
            base.followPath(pathChain, maxPower, holdEnd);
        }

        public double getDriveError() {
            return base.getDriveError();
        }

        public Vector getTeleopHeadingVector() {
            return base.getTeleopHeadingVector();
        }

        public void update() {
            base.update();
        }

        public void pausePathFollowing() {
            base.pausePathFollowing();
        }

        public boolean getUseHeading() {
            return base.getUseHeading();
        }

        public void setTranslationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) {
            base.setTranslationalPIDFCoefficients(translationalPIDFCoefficients);
        }

        public PathPoint getClosestPose() {
            return base.getClosestPose();
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric) {
            base.setTeleOpDrive(forward, strafe, turn, isRobotCentric);
        }

        public void turnTo(double radians) {
            base.turnTo(radians);
        }

        public void activateTranslational() {
            base.activateTranslational();
        }

        public boolean isTurning() {
            return base.isTurning();
        }

        public double getCurrentPathNumber() {
            return base.getCurrentPathNumber();
        }

        public boolean getTeleopDrive() {
            return base.getTeleopDrive();
        }

        public PathPoint getPreviousClosestPose() {
            return base.getPreviousClosestPose();
        }

        public FollowerConstants getConstants() {
            return base.getConstants();
        }

        public void setHeadingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) {
            base.setHeadingPIDFCoefficients(headingPIDFCoefficients);
        }

        public void setXVelocity(double vel) {
            base.setXVelocity(vel);
        }

        public void updateVectors() {
            base.updateVectors();
        }

        public double getCentripetalScaling() {
            return base.getCentripetalScaling();
        }

        public Vector getHeadingVector() {
            return base.getHeadingVector();
        }

        public void setPose(Pose pose) {
            base.setPose(pose);
        }

        public PathBuilder pathBuilder() {
            return base.pathBuilder();
        }

        public double getClosestPointHeadingGoal() {
            return base.getClosestPointHeadingGoal();
        }

        public double getAngularVelocity() {
            return base.getAngularVelocity();
        }

        public void setSecondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) {
            base.setSecondaryTranslationalPIDFCoefficients(secondaryTranslationalPIDFCoefficients);
        }

        public void turn(double radians, boolean isLeft) {
            base.turn(radians, isLeft);
        }

        public Vector getTranslationalError() {
            return base.getTranslationalError();
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, double offsetHeading) {
            base.setTeleOpDrive(forward, strafe, turn, offsetHeading);
        }

        public void setMaxPowerScaling(double maxPowerScaling) {
            base.setMaxPowerScaling(maxPowerScaling);
        }

        public double getPathCompletion() {
            return base.getPathCompletion();
        }

        public void setStartingPose(Pose pose) {
            base.setStartingPose(pose);
        }

        public boolean isTeleopDrive() {
            return base.isTeleopDrive();
        }

        public void setConstraints(PathConstraints pathConstraints) {
            base.setConstraints(pathConstraints);
        }

        public double getHeading() {
            return base.getHeading();
        }

        public void turnToDegrees(double degrees) {
            base.turnToDegrees(degrees);
        }

        public void activateHeading() {
            base.activateHeading();
        }

        public void resumePathFollowing() {
            base.resumePathFollowing();
        }

        public boolean isBusy() {
            return base.isBusy();
        }

        public boolean getUseDrive() {
            return base.getUseDrive();
        }

        public PoseTracker getPoseTracker() {
            return base.getPoseTracker();
        }

        public PathConstraints getConstraints() {
            return base.getConstraints();
        }

        public void setCentripetalScaling(double set) {
            base.setCentripetalScaling(set);
        }

        public void startTeleopDrive(boolean useBrakeMode) {
            base.startTeleopDrive(useBrakeMode);
        }

        public void holdPoint(Pose pose) {
            base.holdPoint(pose);
        }

        public double getCurrentTValue() {
            return base.getCurrentTValue();
        }

        public boolean getUseCentripetal() {
            return base.getUseCentripetal();
        }

        public void turnDegrees(double degrees, boolean isLeft) {
            base.turnDegrees(degrees, isLeft);
        }

        public void deactivateAllPIDFs() {
            base.deactivateAllPIDFs();
        }

        public Vector getCorrectiveVector() {
            return base.getCorrectiveVector();
        }

        public Pose getPose() {
            return base.getPose();
        }

        public boolean getFollowingPathChain() {
            return base.getFollowingPathChain();
        }

        public void setYVelocity(double vel) {
            base.setYVelocity(vel);
        }

        public Vector getAcceleration() {
            return base.getAcceleration();
        }

        public void followPath(Path path) {
            base.followPath(path);
        }

        public void updateErrors() {
            base.updateErrors();
        }

        public void breakFollowing() {
            base.breakFollowing();
        }

        public double getMaxPowerScaling() {
            return base.getMaxPowerScaling();
        }

        public double getDistanceTraveledOnPath() {
            return base.getDistanceTraveledOnPath();
        }

        public void holdPoint(BezierPoint point, double heading, boolean useHoldScaling) {
            base.holdPoint(point, heading, useHoldScaling);
        }

        public boolean isLocalizationNAN() {
            return base.isLocalizationNAN();
        }

        public Vector getVelocity() {
            return base.getVelocity();
        }

        public void setConstants(FollowerConstants constants) {
            base.setConstants(constants);
        }

        public PoseHistory getPoseHistory() {
            return base.getPoseHistory();
        }

        public void activateDrive() {
            base.activateDrive();
        }

        public Drivetrain getDrivetrain() {
            return base.getDrivetrain();
        }

        public double getHeadingError() {
            return base.getHeadingError();
        }

        // --------------------AprilTag Re-localization Method--------------------------------
        @SuppressLint("DefaultLocale")
        public boolean relocalize(Telemetry telemetry) {
            // Update the pinpoint velocity estimate as normal
            base.updatePose();
            robotVel = base.getVelocity();
            robotAngVel = base.getAngularVelocity();
            Vector vel = robotVel;
            double angVel = robotAngVel;

            // ------------------------Exit conditions-----------------
            // Don't relocalize if robot moving too fast (motion blur)
            if (vel.getMagnitude() > 1.0 || Math.toDegrees(angVel) > 1.0) {
                return false; // EXIT METHOD
            }

            // Get camera's pose (field-relative) using AprilTag
            Pose poseWorldTurret = camera.update(telemetry);
            // If no pose was found, default to the pinpoint localizer
            if (poseWorldTurret == null){
                return false; // EXIT METHOD
            }

            // ------------------------Continue------------------------
            // Add 90 degrees to change AprilTag heading information to same orientation as robot.
            poseWorldTurret = new Pose(poseWorldTurret.getX(), poseWorldTurret.getY(),poseWorldTurret.getHeading() + Math.PI/2);

            // Transforming the turret pose into the robot (FIELD RELATIVE) pose--> Pose multiplication: pWR = pWT * pRT^-1.
            // The calculation above is now unnecessary because the camera is mounted on the robot, not the turret
            Pose poseWorldRobot = poseWorldTurret/*.times(turret.getPoseRobotTurret().inverse())*/;

            // Set the localizer pose to the current field-relative pose based on AprilTag reading.
            base.setPose(poseWorldRobot);

            // Telemetry displays robot position and heading information
            telemetry.addLine(String.format("Pose XY %6.1f %6.1f  (inch)",
                    poseWorldRobot.getX(),
                    poseWorldRobot.getY()));
            telemetry.addLine(String.format("Pose Heading %6.1f  (rad)",
                    poseWorldRobot.getHeading()
            ));

            return true; // Returns true for successful relocalization
        }

//        // ---------------------------Draw Robot Method---------------------------------------
//        public void drawPose(@NonNull TelemetryPacket p){
//            p.fieldOverlay().setStroke("#3F51B5");
//            Drawing.drawRobot(p.fieldOverlay(), localizer.getPose());
//            p.fieldOverlay().setStroke("#00FF00");
//            Drawing.drawRobot(p.fieldOverlay(),localizer.getPose().times(RobotConstants.turretPos));
//        }

    } // END OF APRILDRIVE CLASS


    // ---------------------------AUTON INITIALIZATION----------------------------------------------
    // Initialize and set motors, shooter, timers
    public void initAuto(HardwareMap hardwareMap, Telemetry telemetry, OpModeState opModeState) {
        if (opModeState == OpModeState.AUTO_FAR){
            Positions.goalPos = handleVector(RobotConstants.goalPosFar);
            Positions.deltaH = RobotConstants.DELTA_H_FAR;
        }
        intake = new Intake(hardwareMap);
//        camera = new AprilTagLocalization2(hardwareMap);
        camera = new Limelight(hardwareMap);
        follower = new AprilFollower(org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap));
        follower.setPose(txWorldPinpoint.getAsCoordinateSystem(PedroCoordinates.INSTANCE));
//        follower = new AprilFollower(hardwareMap, txWorldPinpoint);
        shooter = new Shooter(hardwareMap);
        ballDetector = new BallDetector(hardwareMap);

        this.opModeState = opModeState;
        launchState = LaunchState.IDLE;
        driveState = DriveState.RELATIVE;

        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();

        Positions.drivePower = RobotConstants.DRIVE_POWER;
        Positions.turretClip0 = RobotConstants.TURRET_CLIP_0;
        Positions.turretClip1 = RobotConstants.TURRET_CLIP_1;
        Positions.flywheelPower = RobotConstants.FLYWHEEL_POWER;
        Positions.flywheelPowerOffset = 0;
        Positions.turretAngleManualOffset = 0;
        Positions.hoodAngleManualOffset = 0;
        RobotConstants.TURRET_OFFSET = 0;
        RobotConstants.HOOD_OFFSET = 0;

//        if (!Constants.MINIMIZE_TELEMETRY) {
//            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//            RobotLog.dd("auto shot pose", " " + Positions.autoShotPose.position.x + " " + Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
//            RobotLog.dd("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " + Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
//            RobotLog.dd("tele shot pose", " " + Positions.teleShotPose.position.x + " " + Positions.teleShotPose.position.y + " " + Positions.teleShotPose.heading.toDouble());
//            RobotLog.dd("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//            RobotLog.dd("deltaH", " " + Positions.deltaH);
//        }
//        telemetry.addData("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//        telemetry.addData("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " +  Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
//        telemetry.addData("tele shot pose", " " + Positions.teleShotPose .position.x + " " +  Positions.teleShotPose .position.y + " " + Positions.teleShotPose .heading.toDouble());
//        telemetry.addData("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//        telemetry.addData("auto shot pose", " " + Positions.autoShotPose.position.x + " " +  Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
//        telemetry.addData("deltaH"," "+ Positions.deltaH);
//        telemetry.addData("Status", "Initialized"); //IMPORTANT
//        telemetry.update();
    } // END OF INITIALIZATION FUNCTION

    // -----------------------TELEOP INITIALIZATION-------------------------------------------------
    public void initTeleop(HardwareMap hardwareMap, Telemetry telemetry) {
//        RobotConstants.turretTargetRangeOffset = (Constants.turretLowAngle + Constants.turretHighAngle )/2.0;
        // Initialize subsystems
        intake = new Intake(hardwareMap);
//        camera = new AprilTagLocalization2(hardwareMap);
        camera = new Limelight(hardwareMap);
        follower = new AprilFollower(org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap));
        follower.setPose(txWorldPinpoint.getAsCoordinateSystem(PedroCoordinates.INSTANCE));
        shooter = new Shooter(hardwareMap);
        ballDetector = new BallDetector(hardwareMap);


        // Set enums
        opModeState = OpModeState.TELEOP;
        launchState = LaunchState.IDLE;
        driveState = DriveState.RELATIVE;

        // Start timers
        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();

        // Constants
        Positions.drivePower = RobotConstants.DRIVE_POWER_TELE;
        Positions.turretClip0 = RobotConstants.TURRET_CLIP_0_TELE;
        Positions.turretClip1 = RobotConstants.TURRET_CLIP_1_TELE; //CHANGE BACK
        Positions.flywheelPower = RobotConstants.FLYWHEEL_POWER; //TELE, if neeeded
        Positions.flywheelPowerOffset = 0;
        Positions.turretAngleManualOffset = 0;
        Positions.hoodAngleManualOffset = 0;
        RobotConstants.TURRET_OFFSET = 0;
        RobotConstants.HOOD_OFFSET = 0;

        // Logs and telemetry
//        if (!Constants.MINIMIZE_TELEMETRY) {
//            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//            RobotLog.dd("auto shot pose", " "
//                    + Positions.autoShotPose.position.x + " "
//                    + Positions.autoShotPose.position.y + " "
//                    + Positions.autoShotPose.heading.toDouble());
//            RobotLog.dd("auto shot pose far", " "
//                    + Positions.autoShotPose_Far.position.x + " "
//                    + Positions.autoShotPose_Far.position.y + " "
//                    + Positions.autoShotPose_Far.heading.toDouble());
//            RobotLog.dd("tele shot pose", " "
//                    + Positions.teleShotPose.position.x + " "
//                    + Positions.teleShotPose.position.y + " "
//                    + Positions.teleShotPose.heading.toDouble());
//            RobotLog.dd("initial pose", " "
//                    + txWorldPinpoint.position.x + " "
//                    + txWorldPinpoint.position.y + " "
//                    + txWorldPinpoint.heading.toDouble());
//            RobotLog.dd("deltaH", " " + Positions.deltaH);
//            telemetry.addData("goal pos", " "
//                    + Positions.goalPos.x + " "
//                    + Positions.goalPos.y);
//            telemetry.addData("auto shot pose far", " "
//                    + Positions.autoShotPose_Far.position.x + " "
//                    + Positions.autoShotPose_Far.position.y + " "
//                    + Positions.autoShotPose_Far.heading.toDouble());
//            telemetry.addData("tele shot pose", " "
//                    + Positions.teleShotPose.position.x + " "
//                    + Positions.teleShotPose.position.y + " "
//                    + Positions.teleShotPose.heading.toDouble());
//            telemetry.addData("initial pose", " "
//                    + txWorldPinpoint.position.x + " "
//                    + txWorldPinpoint.position.y + " "
//                    + txWorldPinpoint.heading.toDouble());
//            telemetry.addData("auto shot pose", " "
//                    + Positions.autoShotPose.position.x + " "
//                    + Positions.autoShotPose.position.y + " "
//                    + Positions.autoShotPose.heading.toDouble());
//            telemetry.addData("deltaH", " " + Positions.deltaH);
//        }
//        telemetry.addData("Status", "Initialized"); // IMPORTANT
//        if (!RobotConstants.MINIMIZE_TELEMETRY) {
//            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//            RobotLog.dd("auto shot pose", " " + Positions.autoShotPose.position.x + " " + Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
//            RobotLog.dd("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " + Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
//            RobotLog.dd("tele shot pose", " " + Positions.teleShotPose.position.x + " " + Positions.teleShotPose.position.y + " " + Positions.teleShotPose.heading.toDouble());
//            RobotLog.dd("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//            RobotLog.dd("deltaH", " " + Positions.deltaH);
//        }
//        telemetry.addData("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//        telemetry.addData("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " +  Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
//        telemetry.addData("tele shot pose", " " + Positions.teleShotPose .position.x + " " +  Positions.teleShotPose .position.y + " " + Positions.teleShotPose .heading.toDouble());
//        telemetry.addData("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//        telemetry.addData("auto shot pose", " " + Positions.autoShotPose.position.x + " " +  Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
//        telemetry.addData("deltaH"," "+ Positions.deltaH);
//        telemetry.addData("Status", "Initialized"); //IMPORTANT
//        RobotLog.d("dirve side sign"+driveSideSign);
//        telemetry.update();
    } // END OF INITIALIZATION FUNCTION

    //temp thing that won't be needed later (follower only for drive testing)
    public void initFollower(HardwareMap hardwareMap, Telemetry telemetry) {
        follower = new AprilFollower(org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap));
        // Set enums
        opModeState = OpModeState.TELEOP;
        launchState = LaunchState.IDLE;
        driveState = DriveState.RELATIVE;

        // Start timers
        feederTimer = new ElapsedTime();
        aprilTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();

        // Constants
        Positions.drivePower = RobotConstants.DRIVE_POWER_TELE;
        Positions.turretClip0 = RobotConstants.TURRET_CLIP_0_TELE;
        Positions.turretClip1 = RobotConstants.TURRET_CLIP_1_TELE; //CHANGE BACK
        Positions.flywheelPower = RobotConstants.FLYWHEEL_POWER; //TELE ,if needed
        Positions.flywheelPowerOffset = 0;
        Positions.turretAngleManualOffset = 0;
        Positions.hoodAngleManualOffset = 0;
        RobotConstants.TURRET_OFFSET = 0;
        RobotConstants.HOOD_OFFSET = 0;

        // Logs and telemetry
//        if (!Constants.MINIMIZE_TELEMETRY) {
//            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//            RobotLog.dd("auto shot pose", " "
//                    + Positions.autoShotPose.position.x + " "
//                    + Positions.autoShotPose.position.y + " "
//                    + Positions.autoShotPose.heading.toDouble());
//            RobotLog.dd("auto shot pose far", " "
//                    + Positions.autoShotPose_Far.position.x + " "
//                    + Positions.autoShotPose_Far.position.y + " "
//                    + Positions.autoShotPose_Far.heading.toDouble());
//            RobotLog.dd("tele shot pose", " "
//                    + Positions.teleShotPose.position.x + " "
//                    + Positions.teleShotPose.position.y + " "
//                    + Positions.teleShotPose.heading.toDouble());
//            RobotLog.dd("initial pose", " "
//                    + txWorldPinpoint.position.x + " "
//                    + txWorldPinpoint.position.y + " "
//                    + txWorldPinpoint.heading.toDouble());
//            RobotLog.dd("deltaH", " " + Positions.deltaH);
//            telemetry.addData("goal pos", " "
//                    + Positions.goalPos.x + " "
//                    + Positions.goalPos.y);
//            telemetry.addData("auto shot pose far", " "
//                    + Positions.autoShotPose_Far.position.x + " "
//                    + Positions.autoShotPose_Far.position.y + " "
//                    + Positions.autoShotPose_Far.heading.toDouble());
//            telemetry.addData("tele shot pose", " "
//                    + Positions.teleShotPose.position.x + " "
//                    + Positions.teleShotPose.position.y + " "
//                    + Positions.teleShotPose.heading.toDouble());
//            telemetry.addData("initial pose", " "
//                    + txWorldPinpoint.position.x + " "
//                    + txWorldPinpoint.position.y + " "
//                    + txWorldPinpoint.heading.toDouble());
//            telemetry.addData("auto shot pose", " "
//                    + Positions.autoShotPose.position.x + " "
//                    + Positions.autoShotPose.position.y + " "
//                    + Positions.autoShotPose.heading.toDouble());
//            telemetry.addData("deltaH", " " + Positions.deltaH);
//        }
//        telemetry.addData("Status", "Initialized"); // IMPORTANT
//        if (!Constants.MINIMIZE_TELEMETRY) {
//            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//            RobotLog.dd("auto shot pose", " " + Positions.autoShotPose.position.x + " " + Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
//            RobotLog.dd("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " + Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
//            RobotLog.dd("tele shot pose", " " + Positions.teleShotPose.position.x + " " + Positions.teleShotPose.position.y + " " + Positions.teleShotPose.heading.toDouble());
//            RobotLog.dd("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//            RobotLog.dd("deltaH", " " + Positions.deltaH);
//        }
//        telemetry.addData("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//        telemetry.addData("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " +  Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
//        telemetry.addData("tele shot pose", " " + Positions.teleShotPose .position.x + " " +  Positions.teleShotPose .position.y + " " + Positions.teleShotPose .heading.toDouble());
//        telemetry.addData("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//        telemetry.addData("auto shot pose", " " + Positions.autoShotPose.position.x + " " +  Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
//        telemetry.addData("deltaH"," "+ Positions.deltaH);
//        telemetry.addData("Status", "Initialized"); //IMPORTANT
//        RobotLog.d("dirve side sign"+driveSideSign);
//        telemetry.update();
    } // END OF INITIALIZATION FUNCTION

    // --------------------WHEN TELEOP STARTS-------------------------------------------------------
    public void startTeleop(){
    }


    // ---------------------------ROBOT CONSTANTS CLASS---------------------------------------------
    public static class Constants{
        public final static boolean MINIMIZE_TELEMETRY = false;
        public final static boolean friedFeed = true;
        public static double flywheelPower = 2.32;
        public static double flywheelPower_Tele = 2.32;
        public static double hoodAngleOffset = 0;
        public final static double deltaH = 39; //height of wall = 54, height of ball exit = 18 inch - 3 inch //32; //height of turret = 8.436535433
        public final static double deltaH_far = 41;
        // GOAL POSES
        public final static Vector goalPos = new Vector(Math.hypot(-58.3727-5.3,55.6425+5.3), Math.atan2(55.6425+5.3,-58.3727-5.3)); //distance to triangle center = 7.5 inch
        public final static Vector goalPos_far = new Vector(Math.hypot(-70,55.6425+12), Math.atan2(55.6425+12,-70)); //distance to triangle center = 7.5 inch
        // SHOT POSES
        public final static Pose autoShotPose = new Pose(-43,33,Math.toRadians(90)); //new Pose2d(-12,15,Math.toRadians(90));
        public final static Pose autoShotPose_Far = new Pose(56,12,Math.toRadians(120));
        public final static Pose teleShotPose = new Pose(0,0,goalPos.getTheta()+Math.PI);
        public final static Pose teleShotPose_Far = new Pose(0,0,goalPos.getTheta()+Math.PI);
        // FLYWHEEL
        public final static double spinUpTimeout = 2;
        public final static double kP = 0.052, kI = 0, kD = 0.000, kF = 10 /*kF will be removed in our new version*/, kS = 0.65, kV = 0.00450;
        // FEEDER
        public final static double feederPower = 1.0;
        public final static double slowIntakePower = 0.7;
        public final static double intakePower = 1.0;
        public final static double outtakePower = 0.6;
        public final static double intakePulseTime = 0.55;
        public final static double outtakePulseTime = 0.15;
        public final static double outtakePulseTime_Tele = 0.15;
        public final static double intakeStabilizeTime = 0.4;
        public final static double feedTime = 0.5; //0.5; //probably increase to 0.6
        public final static double feedDownTime = 0.4; //less than feedUpTime probably
        // HOOD
        public final static double hoodLowAngle = 65.36742754 * Math.PI/180; // the traj angle from horizonatla (rad) //75 //0;
        public final static double hoodHighAngle = 24.47717215 * Math.PI/180; //30 //50*Math.PI/180; //the traj angle from horizontal 55; // Highest actual degree is 41
        public final static double hoodScale0 = 0.2; //0.27;
        public final static double hoodScale1 = 0.8; //1; //0.85;
        // TURRET
        /**
         * turret 0 = 0.48
         * oldest ones also work
         * for backwards turret, add math.pi here
         */
        public static double turretAngleOffset = 0;
        public final static Pose turretPos = new Pose(-1.512,-0.12224409,0);
        public final static double turretHighAngle = 3*Math.PI/2; //164.7*Math.PI/180; //220*Math.PI/180;; //355*Math.PI/180; // //140*Math.PI/180; // In rad, pos = 1
        public final static double turretLowAngle = Math.PI/2; //-175*Math.PI/180; //-40*Math.PI/180;; // //-208*Math.PI/180; // In rad (= old -330 deg)
//        public final static double turretTargetRangeOffset = turretHighAngle-Math.PI; //offset from (-pi,pi)
        // Offset from (-pi,pi) to (midpoint-pi, midpoint+pi), i.e. shift midpoint from 0 to new midpoint
        public final static double turretTargetRangeOffset = Math.PI/2; //(turretLowAngle + turretHighAngle )/2.0; //turretHighAngle-Math.PI;
        public final static double turretScale0 = 0.33055555555555555; //0.218; //0; //0.25 ;//0; //0.11;
        public final static double turretScale1 = 0.7838888888888889; //0.67; //1; //0.78; //0.86; //1;
        public final static double turretClip0 = 0;
        public final static double turretClip0_tele = turretScale0;
        public final static double turretClip1 = 0.8;
        public final static double turretClip1_tele = turretScale1;
        public final static double feederScale0 = 0;
        public final static double feederScale1 = 1;
        public final static double drivePower = 1.0;
        public final static double drivePower_Tele = 1.0;
        public final static double drivePower_Slow = 0.2;
    } // END OF CONSTANTS CLASS


    // ---------------------------POSITIONS CLASS---------------------------------------------------
    // These are mobile positions, which are mirrored depending on color. Also includes scalars that are toggled between different values. For better code readability and error proofing.
    public static class Positions{
        public static Vector goalPos = Constants.goalPos;
        // Where the robot will shoot from:
        public static Pose autoShotPose = Constants.autoShotPose;
        public static Pose autoShotPose_Far = Constants.autoShotPose_Far;
        public static Pose teleShotPose = Constants.teleShotPose;
        public static double deltaH = Constants.deltaH;
        public static double drivePower = Constants.drivePower;
        public static double turretClip0 = Constants.turretClip0;
        public static double turretClip1 = Constants.turretClip1;
        public static double flywheelPower = Constants.flywheelPower;
        public static double flywheelPowerOffset = 0;
        public static double turretAngleManualOffset = 0;
        public static double hoodAngleManualOffset = 0;
    } // END OF POSITIONS CLASS


    // ---------------------------TELEOP FIELD-CENTRIC DRIVING--------------------------------------
    public boolean driveFieldCentric(double forward, double right, double rotate, boolean slowMode, boolean p2p) {
        // Normal vs. slow mode
        if (!p2p) {
            if (slowMode) {
                Positions.drivePower = RobotConstants.DRIVE_POWER_SLOW;
            } else {
                Positions.drivePower = RobotConstants.DRIVE_POWER_TELE;
            }
            follower.setTeleOpDrive(
                    -forward,
                    -right,
                    -rotate,
                    true // Robot Centric
            );
        }
        else{
            follower.holdPoint(follower.getPose());
        }
        return false;
    } // END OF DRIVING FUNCTION

    //temp testing method delete it later
    //use txWorldPinpoint instead of follower.getPose() for like every method thinkable
    //shoter calculations are all done in the InvertedFTCCoordinates coordinate system. to be consistent with old code and camera coordinates
    public void calculateShooter(Telemetry telemetry, boolean moveShot){
        // --------------------Getting robot pose and velocity----------------------
        Pose poseRobot = txWorldPinpoint; // Robot pose
//        poseRobot.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
//        Pose2D poseRobot = PoseConverter.poseToPose2D(pedroPose, InvertedFTCCoordinates.INSTANCE);
        Vector linVel = robotVel;   // in in/s (consistent with other code)
        double angVel = robotAngVel;         // rad/s

        // ------------------------Shooting trajectory values-----------------------
        double p = 0.65; // Fraction of time along trajectory from ground to ground
        double g = 386.22; // Gravity (in/s^2)
        double deltaH = Positions.deltaH; // Height difference from shooter to goal
        double flightTime = Math.sqrt(2 * deltaH / (p * g * (1 - p))); // Ball trajectory time from ground to ground

        // ---------------------------Moving shot correction------------------------
        // angVel thing is incorrect, the heading should be same as original but the turning should offset the virtual pose a tiny bit more.
        // this thing is super wrong right now
        Pose correctedPoseRobot = new Pose(poseRobot.getAsVector().plus(linVel.times(flightTime)).getXComponent(),poseRobot.getAsVector().plus(linVel.times(flightTime)).getYComponent(),poseRobot.getHeadingAsUnitVector().times(angVel*flightTime).getMagnitude());
        if (moveShot){
            poseRobot = correctedPoseRobot;
        }


        // Calculate shot vector using TURRET's position on the robot and ROBOT's heading---
        Pose pose = poseRobot.plus(RobotConstants.turretPos); // Turret's field-relative position
        Vector goalVector = Positions.goalPos.minus(pose.getAsVector());
        double distance = goalVector.getMagnitude(); // Horizontal distance
        double goalVectorAngle = goalVector.getTheta(); // Angle

        // -----Calculate hood angle, flywheel velocity, and turret angle--------
        theta = Math.atan(deltaH / (distance * (1 - p))); // Ball launch angle of elevation
        double vel = distance / (p * flightTime * Math.cos(theta)); // Ball launch speed
        double heading = pose.getHeading(); //
        turretAngle = goalVectorAngle - heading; // Angle to turn turret to (relative to robot's heading)
        double wheelRadius = 1.89;
        /*
        double wheelCircumference = Math.PI * wheelDiameter;
        double change = 0;
        if (up) change += 0.001;
        if (down) change -= 0.001;
        StarterRobot.Constants.flywheelPower += change;
        */
        // Convert vel (speed) to rad/s (example calibration: vel = wheelRadius * rad/s
        radps = vel / wheelRadius; // velocity in rad per s


        // ---------------------------TELEMETRY LINES--------------------------------
        telemetry.addLine("robotTurretPose (inchxinchxdeg): " + pose.getX()+" "+pose.getY()+" "+pose.getHeading()); //NOT IMPORTANT
        telemetry.addLine("goalVector (inchxinch): " + goalVector.getXComponent()+" "+goalVector.getYComponent());
        telemetry.addLine("goalPos (inchxinch): " + Positions.goalPos.getXComponent()+" "+ Positions.goalPos.getYComponent()); //NOT IMPORTANT
//        if (!RobotConstants.MINIMIZE_TELEMETRY) {
            telemetry.addData("goalVector angle (rad to deg)", Math.toDegrees(goalVectorAngle)); //NOT IMPORTANT
            telemetry.addData("distance to goal (inch)", distance); //NOT IMPORTANT
//        }
        telemetry.addData("turret angle (rad to deg)", turretAngle*180/Math.PI);
        telemetry.addData("hood theta (rad to deg)", theta*180/Math.PI);
        telemetry.addData("hood angle offset (rad to deg)", RobotConstants.HOOD_OFFSET*180/Math.PI);
        telemetry.addData("hood angle offset manual (rad to deg)", Positions.hoodAngleManualOffset*180/Math.PI);
        telemetry.addData("targetVel (rad/s to tick/s)", radps*28/Math.PI/2*(Positions.flywheelPower+ Positions.flywheelPowerOffset));
//        if (!RobotConstants.MINIMIZE_TELEMETRY) {
            telemetry.addData("targetVel (rad/s)", radps); //NOT IMPORTANT
//        }
//        telemetry.update();
    }

    public void updateBallDetector(){
        ballDetector.update();
    }

    // ---------------------------SHOOTER METHOD----------------------------------------------------
    // Calculates and sets hood angle and flywheel RPM. Includes shooter state manager.

    //TODO: still need to incorporate a timeout, and manual override
    //TODO: also move shot and setRobotPose
    public void updateShooter(Telemetry telemetry, boolean spinup, boolean humanFeed, boolean lock, boolean idle, boolean moveShot, Pose setRobotPose, double flywheelChange, boolean hoodUp, boolean hoodDown, boolean turretLeft, boolean turretRight) {
        //basically will decide whether the shooter should be spinning or not, modifying some params before passing into shooter update
        boolean shoot = true;
        switch (launchState){
            case IDLE: //flywheel is powered off and hood/turret are not moving
                //if sees ball at top or intaking is off or transferring or force shot ready ==> SPIN_UP
                shoot = false;
                if (ballDetector.ballPresent(telemetry) || intakeState == IntakeState.TRANSFERRING || spinup || humanFeed){
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP: //flywheel is reaching the desired speed and hood/turret are moving to pos
                //if flywheel speed and hood/turret angles are correct ==> SHOOTING
                if (shooter.isAimed()){
                    launchState = LaunchState.SHOOTING;
                }
                //if idle requested ==> IDLE
                else if (idle){
                    launchState = LaunchState.IDLE;
                }
                break;
            case SHOOTING: //ready to shoot
                //if flywheel speed and hood/turret angles are incorrect ==> SPIN_UP
                if (!shooter.isAimed()){
                    launchState = LaunchState.SPIN_UP;
                }
                //if idle requested or (no ball at top AND not transferring) ==> IDLE
                else if (idle || (!ballDetector.ballPresent(telemetry) && intakeState != IntakeState.TRANSFERRING)){
                    launchState = LaunchState.IDLE;
                }
                break;
        }
        Pose robotPose = txWorldPinpoint;
        if (lock){
            robotPose = setRobotPose;
        }
        else if (moveShot){
            robotPose = shooter.adjustMovementPose(txWorldPinpoint, robotVel, robotAngVel);
        }
        shooter.update(shoot, humanFeed, robotPose, flywheelChange > 0.95, flywheelChange < -0.95, hoodUp, hoodDown, turretLeft, turretRight);
        shooter.sendTelemetry(telemetry);

        // --------------------Getting robot pose and velocity----------------------
//        Pose poseRobot = txWorldPinpoint; // Robot pose
//        Vector linVel = robotVel;   // in in/s (consistent with other code)
//        double angVel = robotAngVel;         // rad/s
//
//        // ------------------------Shooting trajectory values-----------------------
//        double p = 0.65; // Fraction of time along trajectory from ground to ground
//        double g = 386.22; // Gravity (in/s^2)
//        double deltaH = Positions.deltaH; // Height difference from shooter to goal
//        double flightTime = Math.sqrt(2 * deltaH / (p * g * (1 - p))); // Ball trajectory time from ground to ground
//
//        // ---------------------------Moving shot correction------------------------
//        // angVel thing is incorrect, the heading should be same as original but the turning should offset the virtual pose a tiny bit more.
//        // this thing is super wrong right now
//        Pose correctedPoseRobot = new Pose(poseRobot.getAsVector().plus(linVel.times(flightTime)).getXComponent(),poseRobot.getAsVector().plus(linVel.times(flightTime)).getYComponent(),poseRobot.getHeadingAsUnitVector().times(angVel*flightTime).getMagnitude());
//        if (moveShot){
//            poseRobot = correctedPoseRobot;
//        }
//
//        // ----------------------------Locking shooter values-----------------------
//        // if setPose just became true, recompute next loop:
//        // If user wants to lock shooter values, set setPose to true, and give it a Pose setRobotPose
//        if (!setPose) lockStarted = false;
//        if (setPose && setRobotPose != null){
//            poseRobot = setRobotPose; // Calculate shooting values on where we plan to shoot
////            RobotLog.d("Correct auto pose!");
//        }
//        else {
//            if (!RobotConstants.MINIMIZE_TELEMETRY) {
//                if (opModeState == OpModeState.AUTO)
//                    RobotLog.d("This is auto right now, it's bad because it's calculating with the wrong pose");
//            }
//        }
//
//        // Calculate shot vector using TURRET's position on the robot and ROBOT's heading---
//        Pose pose = poseRobot.plus(RobotConstants.turretPos); // Turret's field-relative position
//        Vector goalVector = Positions.goalPos.minus(pose.getAsVector());
//        double distance = goalVector.getMagnitude(); // Horizontal distance
//        double goalVectorAngle = goalVector.getTheta(); // Angle
//
//        // -----Calculate hood angle, flywheel velocity, and turret angle--------
//        if (!setPose || !lockStarted) {
//            if (setPose) lockStarted = true;
//            try {
//                theta = Math.atan(deltaH / (distance * (1 - p))); // Ball launch angle of elevation
//                double vel = distance / (p * flightTime * Math.cos(theta)); // Ball launch speed
//                double heading = pose.getHeading(); //
//                turretAngle = goalVectorAngle - heading; // Angle to turn turret to (relative to robot's heading)
//                double wheelRadius = 1.89;
//            /*
//            double wheelCircumference = Math.PI * wheelDiameter;
//            double change = 0;
//            if (up) change += 0.001;
//            if (down) change -= 0.001;
//            StarterRobot.Constants.flywheelPower += change;
//             */
//                // Convert vel (speed) to rad/s (example calibration: vel = wheelRadius * rad/s
//                radps = vel / wheelRadius; // RPM
//            }
//            catch(ArithmeticException e){
//                if (!RobotConstants.MINIMIZE_TELEMETRY) {
//                    RobotLog.dd("SHOOTER CALC FAILED MATH", e.getMessage());
//                    lockStarted = false;
//                }
//            }
//        }
//
//        // ------Adjusting flywheel velocity and turret angle for far shooting--------
//        if (opModeState == OpModeState.TELEOP_FAR && color == Color.RED) { // Far red
//            if (shotReqFeederType) { // front
//                Positions.flywheelPower = 2.481; //change these for teleop
//                RobotConstants.turretAngleOffset = -4 * Math.PI / 180;
//            }
//            else{
//                Positions.flywheelPower = 2.305;
//                RobotConstants.turretAngleOffset = 3 * Math.PI / 180;
//            }
//        }
//        else if (opModeState == OpModeState.TELEOP_FAR && color == Color.BLUE) { // Far blue
//            if (shotReqFeederType) { // front
//                Positions.flywheelPower = 2.145; //change these for teleop
//                RobotConstants.turretAngleOffset = -2 * Math.PI / 180;
//            }
//            else{
//                Positions.flywheelPower = 2.239;
//                RobotConstants.turretAngleOffset = -4 * Math.PI / 180;
//            }
//        }
//
//        // -------------------------Manual adjustment-------------------------------
//        // Hood
//        double hoodChange = 0;
//        if (hoodUp) hoodChange -= Math.PI/180;
//        if (hoodDown) hoodChange += Math.PI/180;
//        Positions.hoodAngleManualOffset += hoodChange;
//        // Turret
//        double turretChange = 0;
//        if (turretLeft) turretChange -= Math.PI/180;
//        if (turretRight) turretChange += Math.PI/180;
//        Positions.turretAngleManualOffset += turretChange;
//
//        // Flywheel RPM
//        double delta = 0;
//        // If the flywheel power change is a big number, adjust little by little
//        if (Math.abs(flywheelChange) > 0.95) delta = Math.signum(flywheelChange)*0.001;
//        Positions.flywheelPowerOffset += delta;
//
//        //-------------------------Toggles--------------------------------------
//        // Spin reverse direction for human feeding (when button is held down)
//        if (humanFeed){
//            //maybe also keep hood low and turret at constant pos
//            radps = -800 / 28.0 * Math.PI * 2;
//            theta = Math.PI/2;
//            turretAngle = 0;
//        }
//        if (flywheelOff){ //not actually flywheelOff but really changing the disable status
//            flywheel.FLYWHEEL.getMotor().setPower(0);
//            telemetry.addLine("trying to 0");
//        }
//        //----Set turret, flywheel, and hood to calculated positions + offsets----
//        else {
//            flywheel.spinTo(radps * 28 / Math.PI / 2 * (Positions.flywheelPower + Positions.flywheelPowerOffset));
//        }
//        // Set hood angle to theta (convert to servo position)
//        hood.turnToAngle(theta+ RobotConstants.hoodAngleOffset+ Positions.hoodAngleManualOffset);
//        turret.turnToRobotAngle(turretAngle+ RobotConstants.turretAngleOffset+ Positions.turretAngleManualOffset);
//
//
//        // ----------------START OF SHOOTER STATE MANAGER----------------------
//        switch (launchState) {
//            case IDLE: // ------------------------------------------------------
//                if (shotReqAlt /*&& feederTimer.seconds()>RobotConstants.feedTime*/){
//                    // If 2 artifacts to shoot consecutively
//                    if (chainShotCount == 2){
//                        if (feederTimer.seconds() > /*RobotConstants.feedTime + */RobotConstants.intakePulseTime + RobotConstants.intakeStabilizeTime) {
//                            // True = front feeder, False = back feeder
//                            if (shotReqFeederType) launchState = LaunchState.SPIN_UP_FRONT;
//                            else launchState = LaunchState.SPIN_UP_BACK;
////                            shotReqFeederType = !shotReqFeederType; // Alternate feeders
//                            feederTimer.reset();
//                        }
//                        // Stop pulse
//                        else if (feederTimer.seconds() > /*RobotConstants.feedTime +*/ RobotConstants.intakePulseTime) intake.stop();
//                        // Intake pulse to move ball to a spot
//                        else intake.slowIntake();
//                    }
//                    else { // Only shooting one when shotReqAlt is True: normal shooting
//                        if (shotReqFeederType) launchState = LaunchState.SPIN_UP_FRONT;
//                        else launchState = LaunchState.SPIN_UP_BACK;
////                        shotReqFeederType = !shotReqFeederType;
//                        feederTimer.reset();
//                    }
//                }
//                // Normal shooting
//                else if (!shotReqAlt) {
//                    chainShotCount = 1;
//                    shotReqFeederType = false;
//                }
//                if (shotRequestedFront/* && feederTimer.seconds()>RobotConstants.feedTime*/) {// After feeding is done. change req state here too
//                        launchState = LaunchState.SPIN_UP_FRONT;
////                        shotReqFeederType = false; //next RB will be back. These dont matter anymore
//                        feederTimer.reset();
//                }
//                else if (shotRequestedBack/* && feederTimer.seconds()>RobotConstants.feedTime*/) {
//                        launchState = LaunchState.SPIN_UP_BACK;
////                        shotReqFeederType = true;
//                        feederTimer.reset();
//                }
//                break; // END OF IDLE STATE
//
//            case SPIN_UP_FRONT: // SPEED UP FLYWHEEL---------------------
//                if (flywheel.getVel() > radps * 28 / Math.PI / 2 - 50 || feederTimer.seconds() > RobotConstants.spinUpTimeout) {
//                    launchState = LaunchState.FEED_FRONT;
//                }
//                break;
//            case SPIN_UP_BACK: // SPEED UP FLYWHEEL---------------------
//                if (flywheel.getVel() > radps * 28 / Math.PI / 2 - 50 || feederTimer.seconds() > RobotConstants.spinUpTimeout) {
//                    launchState = LaunchState.FEED_BACK;
//                }
//                break;
//            case FEED_FRONT: // FEED BALL------------------------------------
//                intake.pause();
//                feeder.upFR(); // feeder starts
//                feederTimer.reset(); // feeder goes down
//                launchState = LaunchState.LAUNCHING;
//                break;
//            case FEED_BACK: // FEED BALL--------------------------------------
//                intake.pause();
//                feeder.upBL();
//                feederTimer.reset();
//                launchState = LaunchState.LAUNCHING;
//                break;
//            case LAUNCHING: // RESET EVERYTHING---------------------------------
//                if (feederTimer.seconds() > RobotConstants.feedTime) {
//                    launchState = LaunchState.FEED_DOWN;
//                    feeder.downFR();
//                    feeder.downBL();
//                    feederTimer.reset();
//                    shotReqFeederType = !shotReqFeederType;
//                }
//                break;
//            case FEED_DOWN: // RESET EVERYTHING----------------------------------
//                if (feederTimer.seconds() > RobotConstants.feedDownTime) {
//                    launchState = LaunchState.IDLE;
//                    feederTimer.reset();
//                    intake.proceed();
//                    if (shotReqAlt) chainShotCount++;
//                }
//                break;
//        } // --------------------END OF STATE MANAGER--------------------------------


    } // -------------------------------END OF UPDATE SHOOTER---------------------------------------



    // ---------------------------INTAKE METHOD----------------------------------------------------
    //TODO: figure out ball counting
    //TODO: figure out how balls will rest inside the ramp
    public void updateIntake(boolean in, boolean out, boolean stopIn, boolean shoot, Telemetry telemetry) {
        // ---------------START OF SHOOTER STATE MANAGER----------------------
        switch (intakeState) {
            case IDLE: // ------------------------------------------------------
                if (shoot) {
                    intakeState = IntakeState.TRANSFERRING;
                    if (/*launchState == LaunchState.SHOOTING*/ shooter.isAimed()) { //TODO: 0.5s timeout here later
                        intake.intake();
                    }
                } else if (in) {
                    intake.intake();
                    intakeState = IntakeState.INTAKING;
                } else if (out) {
                    intake.outtake();
                    intakeState = IntakeState.OUTTAKING;
                }
//                if (4) {
//                    intake.outtake();
//                    intakeState = IntakeState.OUTTAKING;
//                }
                break; // END OF IDLE STATE

            case INTAKING:
                if (stopIn){
                    intake.stop();
                    intakeState = IntakeState.IDLE;
                }
                if (ballDetector.ballPresent(telemetry)){ //ball present at top. or 3
                    intake.stop();
                    intakeState = IntakeState.IDLE;
                }
//                if (4){
//                    intake.outtake();
//                    intakeState = IntakeState.OUTTAKING;
//                }
                break;
            case OUTTAKING:
                if (!out){
                    intake.stop();
                    intakeState = IntakeState.IDLE;
                }
                break;
            case TRANSFERRING: // FEED BALL------------------------------------
                if (!shoot){
                    intake.stop();
                    intakeState = IntakeState.IDLE;
                }
                break;
        } // --------------------END OF STATE MANAGER--------------------------------
//        telemetry.update();
    } // -------------------------------END OF UPDATE SHOOTER---------------------------------------

    // ----------------------------------LOCALIZATION CLASS-----------------------------------------
    /**
     * Returns field-relative robot pose (calculated using turret pose), or returns Pinpoint-recorded
     * pose if no AprilTag detections. Also displays pose information on telemetry.
     */
    public void updateFollower(boolean relocalize, boolean relocalize2, Telemetry telemetry){
        switch (driveState){
            case RELATIVE:
                follower.update();
                txWorldPinpoint = follower.getPose().getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
                robotVel = follower.getVelocity();
                robotVel.rotateVector(-Math.PI/2);
                robotAngVel = follower.getAngularVelocity();
//                if (aprilTimer.seconds() > 10){
//                    driveState = DriveState.ABSOLUTE;
//                }
                if (relocalize) driveState = DriveState.ABSOLUTE;
                if (relocalize2) driveState = DriveState.ABSOLUTE2;
                break;
            case ABSOLUTE:
                boolean success = follower.relocalize(telemetry);
                if (success) aprilTimer.reset();
                driveState = DriveState.RELATIVE;
                break;
            case ABSOLUTE2:
                Pose newPose = handlePose(new Pose(-51.84,51.2636,Math.toRadians(-54.2651)));
                follower.setPose(newPose);
                txWorldPinpoint = newPose;
                RobotConstants.HOOD_OFFSET = 0;
                RobotConstants.TURRET_OFFSET = 0;
                Positions.turretAngleManualOffset = 0;
                Positions.hoodAngleManualOffset = 0;
                driveState = DriveState.RELATIVE;
                break;
        }
        double curTime = aprilTimer.milliseconds();
        telemetry.addData("loop time (ms)",curTime-lastTime);
        telemetry.addData("time since last relocalization (s)", curTime/1000.0); //IMPORTANT
        lastTime = curTime;
        telemetry.addData("x", follower.getPose().getX()); //ALL IMPORTANT
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
    } //----------------------------END OF LOCALIZATION CLASS---------------------------------------

    // ---------------------------------------AIMING------------------------------------------------
    public boolean setGoalTarget(){
        double x = txWorldPinpoint.getX();
        boolean close = x < 10;
        if (close){
            Positions.goalPos = handleVector(RobotConstants.goalPos);
            Positions.deltaH = RobotConstants.DELTA_H;
            Positions.teleShotPose = handlePose(RobotConstants.teleShotPose);
            opModeState = OpModeState.TELEOP;
        }
        else { //chagne to be better at far
            Positions.goalPos = handleVector(RobotConstants.goalPosFar);
            Positions.deltaH = RobotConstants.DELTA_H_FAR;
            Positions.teleShotPose = handlePose(RobotConstants.teleShotPoseFar);
            opModeState = OpModeState.TELEOP_FAR;
        }
        return close;
    }
//    public double updateVoltage(Telemetry telemetry){
//        double voltage = voltageSensor.updateVoltage();
//        telemetry.addData("volts",voltage);
//        return voltage;
//    }

    //------------------------------INTAKE CONTROL METHOD-------------------------------------------
    //OUTDATED ///!!!!
    public void controlIntake(boolean in, boolean out, boolean stop, boolean tapOut, boolean inSlow, boolean outFast){
        if (true/*intakeTimer.seconds() > RobotConstants.outtakePulseTime_Tele*/){
            if (tapping) {intake.proceed(); tapping = false;}
            if (out) intake.outtake();
            else if (in) intake.intake();
            else if (inSlow) intake.slowIntake();
            else if (stop) intake.stop();
            else if (outFast) intake.strongOuttake();
        }
        if (tapOut){
            intake.shortOuttake();
            intakeTimer.reset();
            tapping = true;
        }
    }

    // ------------------------Update pose variable with robot's current pose-----------------------
    public void setPose(Pose pose){
        txWorldPinpoint = pose;
    }


    // Lookup table (lut)
    /*
    public int[][][] power = {{{1}}};
    public int[][][] angle = {{{1}}};
    public int[] lookUp(Vector2d pos, Vector2d vel){

        return new int[]{power[0][0][0],angle[0][0][0]};
    }
     */

    // --------------------------------HELPER METHODS-----------------------------------------------
    // Mirror a vector
    public Vector mirrorVector(Vector vector){
        vector.setOrthogonalComponents(vector.getXComponent(), -vector.getYComponent());
        return vector;
    }
    // If blue, mirror the pose
    public Pose handlePose(Pose pose){
        if (color == Color.BLUE){
            return pose.mirror();
        }
        return pose;
    }
    // If blue, mirror the vector
    public Vector handleVector(Vector vector){
        if (color == Color.BLUE){
            return mirrorVector(vector);
        }
        return vector;
    }


    // ------------------ CALCULATE COLOR SEQUENCE FROM OBELISK ID ---------------------------------
    public static char[] getDesiredPattern(int obeliskID) {
        switch (obeliskID) {
            case 21: return new char[]{'G','P','P'};
            case 22: return new char[]{'P','G','P'};
            case 23: return new char[]{'P','P', 'G'};
            default: return new char[]{'G','P','P'}; // fallback
        }
    }


    // ------------------------COMPUTE INTAKE FIRING ORDER -----------------------------------------
    // Slots: innermost is 0, middle is 1, one in intake is 2
    // Returns an int[] like [0, 1, 2] for firing order
    public static int[] computeFireOrder(char[] queue, char[] desired) {
        boolean[] used = new boolean[queue.length];
        int[] order = new int[desired.length];
        for (int i = 0; i < desired.length; i++) {
            for (int j = 0; j < queue.length; j++) {
                if (!used[j] && queue[j] == desired[i]) {
                    order[i] = j;
                    used[j] = true;
                    break;
                }
            }
        }
        // Mutate order to enforce physical constraints
        // --- Rule 1: Slot #2 cannot fire first ---
        if (order[0] == 2) {
            // find the first element that isn't 2 and swap
            for (int i = order.length-1; i > 0; i--) {
                if (order[i] != 2) {
                    int temp = order[0];
                    order[0] = order[i];
                    order[i] = temp;
                    break;
                }
            }
        }

        // --- Rule 2: If 1 is followed by 2, change 1 → 0 ---
        for (int i = 0; i < order.length - 1; i++) {
            if (i > 0) {
                if (order[i] == 1 && order[i + 1] == 2) {
                    order[i] = 0;
                }
                if (order[i] == 2 && order[i + 1] == 1) {
                    order[i + 1] = 0;
                }
            }
        }
        return order;
    }
    public static int[] Order = {0,1,2};


    // ------------------------CALCULATE FEEDER FIRING SEQUENCE-------------------------------------
//    public static Action shootSequence(AtomicBoolean shotReqFR, AtomicBoolean shotReqBL, AtomicBoolean slowIntakeAtomic,
//                                       char[] queue, int obeliskID, int shot, OpModeState opModeState, Color color) {
//
//        char[] desired = getDesiredPattern(obeliskID);
//        Order = computeFireOrder(queue, desired);
//        // If feeder is not working, always shoot front one first
//        if (RobotConstants.friedFeed) Order = new int[]{1,2,0};
//        if (!RobotConstants.MINIMIZE_TELEMETRY) {
//            RobotLog.a("Shohtott " + shot);
//            RobotLog.d("Obelisk ID: " + obeliskID);
//            RobotLog.d("balls in robot: ");
//            display(queue);
//            RobotLog.d("balls score order: ");
//            display(desired);
//            RobotLog.d("feeders move order: ");
//            display(Order);
//        }
//        ArrayList<Action> actions = new ArrayList<>();
//        if (opModeState == OpModeState.AUTO_FAR && color == Color.RED) {
//            if (Order[0] == 0) {
//                actions.add(new InstantAction(() -> Positions.flywheelPower = 2.15));
//                actions.add(new InstantAction(() -> RobotConstants.turretAngleOffset = -1 * Math.PI / 180));
//            } else if (Order[0] == 1 || Order[0] == 2) {
//                actions.add(new InstantAction(() -> Positions.flywheelPower = 2.338));
//                actions.add(new InstantAction(() -> RobotConstants.turretAngleOffset = 5 * Math.PI / 180));
//            }
//        }
//        else if (opModeState == OpModeState.AUTO_FAR && color == Color.BLUE) {
////            RobotLog.a("change first");
//            if (Order[0] == 0) {
//                actions.add(new InstantAction(() -> Positions.flywheelPower = 2.25));
//                actions.add(new InstantAction(() -> RobotConstants.turretAngleOffset = 6 * Math.PI / 180));
//            } else if (Order[0] == 1 || Order[0] == 2) {
//                actions.add(new InstantAction(() -> Positions.flywheelPower = 2.414));
//                actions.add(new InstantAction(() -> RobotConstants.turretAngleOffset = 8 * Math.PI / 180));
//            }
//        }
//        if (opModeState == OpModeState.AUTO || opModeState == OpModeState.AUTO_FAR) {
//            actions.add(new SleepAction(0.4));
//        }
//        for (int i = 0; i <= 2; i++) {
//            int feeder = Order[i]; // Go through the firing order (eg. [0, 1, 2]) and set shot requests to true
//            //TODO: modify for an intake pulse when firing slot 2. This will cause slot 1 to be displaced to slot 0. (push it in one slot)
//            //The intake pulse has been changed to always occur before the second shot. This deterministically causes slot 2 to go to slot 1 and slot 1 to go to slot 0.
//            if (i==1 /*feeder == 2*/) {
//                actions.add(new InstantAction(() -> slowIntakeAtomic.set(true)));
//                actions.add(new SleepAction(RobotConstants.intakePulseTime));
//                actions.add(new InstantAction(() -> slowIntakeAtomic.set(false)));
//                actions.add(new SleepAction(RobotConstants.intakeStabilizeTime));
//            }
//            if (i == 2 && (opModeState == OpModeState.AUTO || opModeState == OpModeState.AUTO_FAR)){
//                actions.add(new SleepAction(0.4));
//            }
//            // then set shotReq booleans and the usual sleep/reset sequence
//            actions.add(new InstantAction(() -> {
//                if (feeder == 1 || feeder == 2) shotReqFR.set(true);  // 1, 2 = front/right feeder
//                else shotReqBL.set(true);               // 0 = back/left feeder
//            }));
//            actions.add(new SleepAction(0.3)); // allow an interval of requesting in case the initial request is overridden
//            actions.add(new InstantAction(() -> {
//                shotReqFR.set(false);
//                shotReqBL.set(false);
//            }));
//            if (opModeState == OpModeState.AUTO_FAR && color == Color.RED && i < 2) {
//                if (Order[i + 1] == 1 || Order[i + 1] == 2) {
//                    actions.add(new SleepAction(0.2));
//                    actions.add(new InstantAction(() -> Positions.flywheelPower = 2.338));
//                    actions.add(new InstantAction(() -> RobotConstants.turretAngleOffset = 5 * Math.PI / 180));
//                } else if (Order[i + 1] == 0) {
//                    actions.add(new SleepAction(0.2));
//                    actions.add(new InstantAction(() -> Positions.flywheelPower = 2.15));
//                    actions.add(new InstantAction(() -> RobotConstants.turretAngleOffset = -1 * Math.PI / 180));
//                }
//            }
//            else if (opModeState == OpModeState.AUTO_FAR && color == Color.BLUE && i < 2) {
////                RobotLog.a("change " + i);
//                if (Order[i + 1] == 0) {
//                    actions.add(new SleepAction(0.2));
//                    actions.add(new InstantAction(() -> Positions.flywheelPower = 2.25));
//                    actions.add(new InstantAction(() -> RobotConstants.turretAngleOffset = 6 * Math.PI / 180));
//                } else if (Order[i + 1] == 1 || Order[i+1] == 2) {
//                    actions.add(new SleepAction(0.2));
//                    actions.add(new InstantAction(() -> Positions.flywheelPower = 2.414));
//                    actions.add(new InstantAction(() -> RobotConstants.turretAngleOffset = 8 * Math.PI / 180));
//                }
//            }
//            if (i != 2) actions.add(new SleepAction(0.6)); // feed`er completes the state machine
//        }
//        SequentialAction seq = new SequentialAction(actions);
//        return seq;
//    }

    public static void display(char[] arr){
        String message = "";
        for (char c : arr) message += c + " ";
        RobotLog.d(message);
    }
    public static void display(int[] arr){
        String message = "";
        for (int j : arr) message += j + " ";
        RobotLog.d(message);
    }
}