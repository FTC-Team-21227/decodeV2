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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake2;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

public class Robot2 {
    private static Robot2 instance = null;
    // SUBSYSTEMS
    Robot2.AprilFollower follower;
    Shooter shooter; // Transfer system (intake), flywheel, turret, and hood
    Limelight limelight; // Limelight subsystem used in AprilDrive and Obelisk detection
    Intake2 intakeTransfer;


    // ATTRIBUTES
    double lastTime = 0; // Previous time, used to calculate loop time
    ElapsedTime aprilTimer; // Time between aprilTag relocalizations
    int driveSideSign = -1; // Alliance color changes forward vs backwards direction
    Pose txWorldPinpoint = new Pose(0,0,Math.PI); // Robot pose based on pinpoint in inverted FTC coordinates (not pedro coordinates)
    Vector robotVel = new Vector(0,0);
    double robotAngVel = 0;


    // ROBOT STATES
    private enum DriveState { // Enum that switches between pinpoint and camera localization
        RELATIVE,
        ABSOLUTE, // Drive with AprilTag localization for absolute position on field
        ABSOLUTE_TELE_RESET
    } private Robot2.DriveState driveState;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        SHOOTING,
    } private LaunchState launchState; // Instance

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
            case ABSOLUTE_TELE_RESET:
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

    // Shooting sequence for one shot
    public void shootSequence(double flywheelVel, double hoodAngle, double turretAngle) {
        ElapsedTime spinUpTimer = new ElapsedTime(); // Time flywheel acceleration
        switch (launchState) {
            case IDLE:
                spinUpTimer.reset();
                intakeTransfer.nextArtifact(); // Push balls to new spots
                launchState = LaunchState.SPIN_UP;
                break;
            case SPIN_UP: // Check if flywheel velocity is enough to shoot
                if(shooter.flywheel.getVel() > flywheelVel - 50 || spinUpTimer.seconds() > Robot.Constants.spinUpTimeout) {
                    launchState = LaunchState.SHOOTING;
                }
                break;
            case SHOOTING:
                intakeTransfer.shootArtifact(); // Move ball to the flywheel
                break;
        }
    }


    // -------------------------APRIL TAG DRIVETRAIN CLASS OF ROBOT---------------------------------
    // Resets localizer using AprilTags. Extends Pedropathing Follower class.
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
    } // ----------------------------------END OF APRILDRIVE CLASS----------------------------------
}