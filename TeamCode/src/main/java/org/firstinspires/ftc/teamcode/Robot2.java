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
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.DynamicPositions;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

public class Robot2 {
    private static Robot2 instance = null;

    // SUBSYSTEMS
    Robot2.AprilFollower follower;
    Shooter shooter; // Transfer system (intake), flywheel, turret, and hood
    Limelight limelight; // Limelight subsystem used in AprilDrive and Obelisk detection


    // ATTRIBUTES
    double lastTime = 0; // Previous time, used to calculate loop time
    ElapsedTime aprilTimer; // Time between aprilTag relocalizations
    int driveSideSign = -1; // Alliance color changes forward vs backwards direction
    Pose txWorldPinpoint = new Pose(0,0,Math.PI); // Stores robot pose based on pinpoint in inverted FTC coordinates (not pedro coordinates)
    Vector robotVel = new Vector(0,0);
    double robotAngVel = 0;


    // ROBOT STATES
    private enum DriveState { // Enum that switches between pinpoint and camera localization
        RELATIVE,
        ABSOLUTE, // Drive with AprilTag localization for absolute position on field
        ABSOLUTE_TELE_RESET
    } private Robot2.DriveState driveState;

    public enum Color { // Enum that stores the alliance color, accessible globally
        RED,
        BLUE
    } public Robot2.Color color;

    public enum OpModeState { // Enum that stores the opmode type, accessible globally
        AUTO,
        AUTO_FAR,
        TELEOP,
        TELEOP_FAR
    } public Robot2.OpModeState opModeState;


    // -------------------PUBLIC ROBOT CONSTRUCTOR--------------------------------------------------
    /**
     * Creates a robot instance with field-relative position starting at initialPose, and mirrors
     * shooting position and goal positions based on alliance color
     * @param initialPose
     * @param color
     */
    public Robot2(Pose initialPose /*in pedro coords*/, Robot2.Color color){
        // HANDLE ALLIANCE COLOR
        this.color = color;
        if (this.color== Robot2.Color.RED) {
            driveSideSign = -1;
            DynamicPositions.goalPos = DynamicPositions.goalPos;
            DynamicPositions.autoShotPose = Constants.autoShotPose;
            DynamicPositions.autoShotPose_Far = Constants.autoShotPose_Far;
            DynamicPositions.teleShotPose = Constants.teleShotPose;
//            RobotPositions.deltaH = RobotConstants.DELTA_H;
            RobotLog.d("It's Red");
        }
        else if (this.color == Robot2.Color.BLUE){
            driveSideSign = 1;
            Robot.Positions.goalPos = mirrorVector(Robot.Constants.goalPos);
            Robot.Positions.autoShotPose = Robot.Constants.autoShotPose.mirror();
            Robot.Positions.autoShotPose_Far = Robot.Constants.autoShotPose_Far.mirror();
            Robot.Positions.teleShotPose = Robot.Constants.teleShotPose.mirror();
//            Robot.Positions.deltaH = Robot.Constants.deltaH;
            RobotLog.d("It's Blue");
        }
        else {
            RobotLog.d("NO COLOR????");
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
    public static Robot2 getInstance(Pose initialPose, Robot2.Color color){
        if (instance == null || instance.opModeState == Robot2.OpModeState.TELEOP){
            RobotLog.d("making a new instance"); // New instance with parameters
            instance = new Robot2(initialPose, color);
        }
        else RobotLog.d("keeping the instance"); // Same instance with same initialPose and color
        return instance;
    }

    // Create new instance
    public static Robot2 startInstance(Pose initialPose, Robot2.Color color){
        instance = new Robot2(initialPose, color);
        return instance;
    }

    // Clear singleton instance
    public static void clearInstance(){
        instance = null;
    }



    // Drives robot field-centric in teleop
    public boolean driveFieldCentric(double forward, double right, double rotate, boolean slowMode, boolean p2p) {
        if (!p2p) { // P2P action?
            // Slow mode?
            if (slowMode) {Robot.Positions.drivePower = Robot.Constants.drivePower_Slow;}
            else {Robot.Positions.drivePower = Robot.Constants.drivePower_Tele;}
            follower.setTeleOpDrive(
                    -forward,
                    -right,
                    -rotate,
                    true // Robot Centric
            );
        }
        else{follower.holdPoint(follower.getPose());} // Stay at point
        return false;
    }


    // Teleop Relocalization method
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
                if (relocalize) driveState = Robot2.DriveState.ABSOLUTE;
                if (relocalize2) driveState = Robot2.DriveState.ABSOLUTE_TELE_RESET;
                break;
            case ABSOLUTE:
                boolean success = follower.relocalize(telemetry);
                if (success) aprilTimer.reset();
                driveState = Robot2.DriveState.RELATIVE;
                break;
            case ABSOLUTE_TELE_RESET: // Reset location against goal at the start of Teleop
                Pose newPose = handlePose(new Pose(-51.84,51.2636,Math.toRadians(-54.2651)));
                follower.setPose(newPose);
                txWorldPinpoint = newPose;
                Robot.Constants.hoodAngleOffset = 0;
                Robot.Constants.turretAngleOffset = 0;
                Robot.Positions.turretAngleManualOffset = 0;
                Robot.Positions.hoodAngleManualOffset = 0;
                driveState = Robot2.DriveState.RELATIVE;
                break;
        }
        double curTime = aprilTimer.milliseconds();
        telemetry.addData("loop time (ms)",curTime-lastTime);
        telemetry.addData("time since last relocalization (s)", curTime / 1000.0); //IMPORTANT
        lastTime = curTime;
        telemetry.addData("x", follower.getPose().getX()); //ALL IMPORTANT
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
    }


    // Set Teleop Goal Position
    public boolean setTeleGoalPos(){
        // Use pinpoint location to determine whether robot is close or far from goal
        double x = txWorldPinpoint.getX();
        boolean close = x < 10;
        if (close){
            Robot.Positions.goalPos = handleVector(Robot.Constants.goalPos);
            Robot.Positions.deltaH = Robot.Constants.deltaH;
            Robot.Positions.teleShotPose = handlePose(Robot.Constants.teleShotPose);
            opModeState = Robot2.OpModeState.TELEOP;
        }
        else { //change to be better at far
            Robot.Positions.goalPos = handleVector(Robot.Constants.goalPos_far);
            Robot.Positions.deltaH = Robot.Constants.deltaH_far;
            Robot.Positions.teleShotPose = handlePose(Robot.Constants.teleShotPose_Far);
            opModeState = Robot2.OpModeState.TELEOP_FAR;
        }
        return close;
    }


    // TODO: Teleop intake control method. ?-How will this work with shooter? will have to figure out
    public void controlIntake(boolean in, boolean out, boolean stop, boolean tapOut, boolean inSlow, boolean outFast){

    }


    // --------------------------------HELPER METHODS-----------------------------------------------
    // Mirror a vector
    public Vector mirrorVector(Vector vector){
        vector.setOrthogonalComponents(vector.getXComponent(), -vector.getYComponent());
        return vector;
    }
    // If blue, mirror the pose
    public Pose handlePose(Pose pose){
        if (color == Robot2.Color.BLUE){return pose.mirror();}
        return pose;
    }
    // If blue, mirror the vector
    public Vector handleVector(Vector vector){
        if (color == Robot2.Color.BLUE){return mirrorVector(vector);}
        return vector;
    }

    // Return color sequence based on obelisk ID #
    public static char[] getDesiredPattern(int obeliskID) {
        switch (obeliskID) {
            case 21: return new char[]{'G','P','P'};
            case 22: return new char[]{'P','G','P'};
            case 23: return new char[]{'P','P', 'G'};
            default: return new char[]{'G','P','P'}; // fallback
        }
    }


    // -------------------------APRIL TAG DRIVETRAIN CLASS OF ROBOT---------------------------------
    // Resets localizer using AprilTags. Extends Pedropathing Follower class.
    public class AprilFollower{
        private final Follower robotBase;
        public AprilFollower(Follower robotBase) {
            this.robotBase = robotBase;
        } // Constructor

        public void updateConstants() {robotBase.updateConstants();}

        public void followPath(Path path, boolean holdEnd) {robotBase.followPath(path, holdEnd);}

        public void startTeleOpDrive(boolean useBrakeMode) {robotBase.startTeleOpDrive(useBrakeMode);}

        public void updatePose() {
            robotBase.updatePose();
        }

        public Vector getCentripetalForceCorrection() {return robotBase.getCentripetalForceCorrection();}

        public void activateAllPIDFs() {
            robotBase.activateAllPIDFs();
        }

        public void setMaxPower(double set) {
            robotBase.setMaxPower(set);
        }

        public void followPath(PathChain pathChain, boolean holdEnd) {robotBase.followPath(pathChain, holdEnd);}

        public boolean atPose(Pose pose, double xTolerance, double yTolerance) {return robotBase.atPose(pose, xTolerance, yTolerance);}

        public double getHeadingGoal(double t) {
            return robotBase.getHeadingGoal(t);
        }

        public Path getCurrentPath() {
            return robotBase.getCurrentPath();
        }

        public String[] debug() {
            return robotBase.debug();
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric, double offsetHeading) {
            robotBase.setTeleOpDrive(forward, strafe, turn, isRobotCentric, offsetHeading);
        }

        public PathChain getCurrentPathChain() {
            return robotBase.getCurrentPathChain();
        }

        public void setDrivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) {
            robotBase.setDrivePIDFCoefficients(drivePIDFCoefficients);
        }

        public void activateCentripetal() {
            robotBase.activateCentripetal();
        }

        public Vector getTeleopDriveVector() {
            return robotBase.getTeleopDriveVector();
        }

        public VectorCalculator getVectorCalculator() {
            return robotBase.getVectorCalculator();
        }

        public void followPath(PathChain pathChain) {
            robotBase.followPath(pathChain);
        }

        public void updateCallbacks() {
            robotBase.updateCallbacks();
        }

        public boolean isRobotStuck() {
            return robotBase.isRobotStuck();
        }

        public Vector getDriveVector() {
            return robotBase.getDriveVector();
        }

        public void holdPoint(BezierPoint point, double heading) {
            robotBase.holdPoint(point, heading);
        }

        public void setTeleOpDrive(double forward, double strafe, double turn) {
            robotBase.setTeleOpDrive(forward, strafe, turn);
        }

        public boolean getUseTranslational() {
            return robotBase.getUseTranslational();
        }

        public void setSecondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) {
            robotBase.setSecondaryHeadingPIDFCoefficients(secondaryHeadingPIDFCoefficients);
        }

        public void startTeleopDrive() {
            robotBase.startTeleopDrive();
        }

        public boolean atParametricEnd() {
            return robotBase.atParametricEnd();
        }

        public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
            return robotBase.atPose(pose, xTolerance, yTolerance, headingTolerance);
        }

        public Vector getClosestPointTangentVector() {
            return robotBase.getClosestPointTangentVector();
        }

        public int getChainIndex() {
            return robotBase.getChainIndex();
        }

        public void updateDrivetrain() {
            robotBase.updateDrivetrain();
        }

        public Pose getPointFromPath(double t) {
            return robotBase.getPointFromPath(t);
        }

        public PathBuilder pathBuilder(PathConstraints constraints) {
            return robotBase.pathBuilder(constraints);
        }

        public void setSecondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) {
            robotBase.setSecondaryDrivePIDFCoefficients(secondaryDrivePIDFCoefficients);
        }

        public void updateErrorAndVectors() {
            robotBase.updateErrorAndVectors();
        }

        public double getDistanceRemaining() {
            return robotBase.getDistanceRemaining();
        }

        public void startTeleOpDrive() {
            robotBase.startTeleOpDrive();
        }

        public double getTotalHeading() {
            return robotBase.getTotalHeading();
        }

        public Vector getTranslationalCorrection() {
            return robotBase.getTranslationalCorrection();
        }

        public ErrorCalculator getErrorCalculator() {
            return robotBase.getErrorCalculator();
        }

        public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
            robotBase.followPath(pathChain, maxPower, holdEnd);
        }

        public double getDriveError() {
            return robotBase.getDriveError();
        }

        public Vector getTeleopHeadingVector() {
            return robotBase.getTeleopHeadingVector();
        }

        public void update() {
            robotBase.update();
        }

        public void pausePathFollowing() {
            robotBase.pausePathFollowing();
        }

        public boolean getUseHeading() {
            return robotBase.getUseHeading();
        }

        public void setTranslationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) {
            robotBase.setTranslationalPIDFCoefficients(translationalPIDFCoefficients);
        }

        public PathPoint getClosestPose() {
            return robotBase.getClosestPose();
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric) {
            robotBase.setTeleOpDrive(forward, strafe, turn, isRobotCentric);
        }

        public void turnTo(double radians) {
            robotBase.turnTo(radians);
        }

        public void activateTranslational() {
            robotBase.activateTranslational();
        }

        public boolean isTurning() {
            return robotBase.isTurning();
        }

        public double getCurrentPathNumber() {
            return robotBase.getCurrentPathNumber();
        }

        public boolean getTeleopDrive() {
            return robotBase.getTeleopDrive();
        }

        public PathPoint getPreviousClosestPose() {
            return robotBase.getPreviousClosestPose();
        }

        public FollowerConstants getConstants() {
            return robotBase.getConstants();
        }

        public void setHeadingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) {
            robotBase.setHeadingPIDFCoefficients(headingPIDFCoefficients);
        }

        public void setXVelocity(double vel) {
            robotBase.setXVelocity(vel);
        }

        public void updateVectors() {
            robotBase.updateVectors();
        }

        public double getCentripetalScaling() {
            return robotBase.getCentripetalScaling();
        }

        public Vector getHeadingVector() {
            return robotBase.getHeadingVector();
        }

        public void setPose(Pose pose) {
            robotBase.setPose(pose);
        }

        public PathBuilder pathBuilder() {
            return robotBase.pathBuilder();
        }

        public double getClosestPointHeadingGoal() {
            return robotBase.getClosestPointHeadingGoal();
        }

        public double getAngularVelocity() {
            return robotBase.getAngularVelocity();
        }

        public void setSecondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) {
            robotBase.setSecondaryTranslationalPIDFCoefficients(secondaryTranslationalPIDFCoefficients);
        }

        public void turn(double radians, boolean isLeft) {
            robotBase.turn(radians, isLeft);
        }

        public Vector getTranslationalError() {
            return robotBase.getTranslationalError();
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, double offsetHeading) {
            robotBase.setTeleOpDrive(forward, strafe, turn, offsetHeading);
        }

        public void setMaxPowerScaling(double maxPowerScaling) {
            robotBase.setMaxPowerScaling(maxPowerScaling);
        }

        public double getPathCompletion() {
            return robotBase.getPathCompletion();
        }

        public void setStartingPose(Pose pose) {
            robotBase.setStartingPose(pose);
        }

        public boolean isTeleopDrive() {
            return robotBase.isTeleopDrive();
        }

        public void setConstraints(PathConstraints pathConstraints) {
            robotBase.setConstraints(pathConstraints);
        }

        public double getHeading() {
            return robotBase.getHeading();
        }

        public void turnToDegrees(double degrees) {
            robotBase.turnToDegrees(degrees);
        }

        public void activateHeading() {
            robotBase.activateHeading();
        }

        public void resumePathFollowing() {
            robotBase.resumePathFollowing();
        }

        public boolean isBusy() {
            return robotBase.isBusy();
        }

        public boolean getUseDrive() {
            return robotBase.getUseDrive();
        }

        public PoseTracker getPoseTracker() {
            return robotBase.getPoseTracker();
        }

        public PathConstraints getConstraints() {
            return robotBase.getConstraints();
        }

        public void setCentripetalScaling(double set) {
            robotBase.setCentripetalScaling(set);
        }

        public void startTeleopDrive(boolean useBrakeMode) {
            robotBase.startTeleopDrive(useBrakeMode);
        }

        public void holdPoint(Pose pose) {
            robotBase.holdPoint(pose);
        }

        public double getCurrentTValue() {
            return robotBase.getCurrentTValue();
        }

        public boolean getUseCentripetal() {
            return robotBase.getUseCentripetal();
        }

        public void turnDegrees(double degrees, boolean isLeft) {
            robotBase.turnDegrees(degrees, isLeft);
        }

        public void deactivateAllPIDFs() {
            robotBase.deactivateAllPIDFs();
        }

        public Vector getCorrectiveVector() {
            return robotBase.getCorrectiveVector();
        }

        public Pose getPose() {
            return robotBase.getPose();
        }

        public boolean getFollowingPathChain() {
            return robotBase.getFollowingPathChain();
        }

        public void setYVelocity(double vel) {
            robotBase.setYVelocity(vel);
        }

        public Vector getAcceleration() {
            return robotBase.getAcceleration();
        }

        public void followPath(Path path) {
            robotBase.followPath(path);
        }

        public void updateErrors() {
            robotBase.updateErrors();
        }

        public void breakFollowing() {
            robotBase.breakFollowing();
        }

        public double getMaxPowerScaling() {
            return robotBase.getMaxPowerScaling();
        }

        public double getDistanceTraveledOnPath() {
            return robotBase.getDistanceTraveledOnPath();
        }

        public void holdPoint(BezierPoint point, double heading, boolean useHoldScaling) {
            robotBase.holdPoint(point, heading, useHoldScaling);
        }

        public boolean isLocalizationNAN() {
            return robotBase.isLocalizationNAN();
        }

        public Vector getVelocity() {
            return robotBase.getVelocity();
        }

        public void setConstants(FollowerConstants constants) {
            robotBase.setConstants(constants);
        }

        public PoseHistory getPoseHistory() {
            return robotBase.getPoseHistory();
        }

        public void activateDrive() {
            robotBase.activateDrive();
        }

        public Drivetrain getDrivetrain() {
            return robotBase.getDrivetrain();
        }

        public double getHeadingError() {
            return robotBase.getHeadingError();
        }

        // --------------------AprilTag Re-localization Method--------------------------------
        @SuppressLint("DefaultLocale")
        public boolean relocalize(Telemetry telemetry) {
            // Update the pinpoint velocity estimate as normal
            robotBase.updatePose();
            robotVel = robotBase.getVelocity();
            robotAngVel = robotBase.getAngularVelocity();
            Vector vel = robotVel;
            double angVel = robotAngVel;

            // Don't relocalize if robot moving too fast (motion blur)
            if (vel.getMagnitude() > 1.0 || Math.toDegrees(angVel) > 1.0) {
                return false; // EXIT METHOD
            }

            // Get camera's pose (field-relative) using AprilTag
            Pose poseWorldTurret = limelight.update(telemetry);
            // If no pose was found, default to the pinpoint localizer
            if (poseWorldTurret == null){
                return false; // EXIT METHOD
            }

            // Add 90 degrees to change AprilTag heading information to same orientation as robot.
            poseWorldTurret = new Pose(poseWorldTurret.getX(), poseWorldTurret.getY(),poseWorldTurret.getHeading() + Math.PI/2);

            // Transforming the turret pose into the robot (FIELD RELATIVE) pose--> Pose multiplication: pWR = pWT * pRT^-1. (Now unnecessary because the camera is mounted on the robot, not the turret)
            Pose poseWorldRobot = poseWorldTurret/*.times(turret.getPoseRobotTurret().inverse())*/;

            // Set the localizer pose to the current field-relative pose based on AprilTag reading.
            robotBase.setPose(poseWorldRobot);

            // Telemetry displays robot position and heading information
            telemetry.addLine(String.format("Pose XY %6.1f %6.1f  (inch)",
                    poseWorldRobot.getX(),
                    poseWorldRobot.getY()));
            telemetry.addLine(String.format("Pose Heading %6.1f  (rad)",
                    poseWorldRobot.getHeading()
            ));

            return true; // Returns true for successful relocalization
        }
    } // ----------------------------------END OF APRILDRIVE CLASS----------------------------------
}