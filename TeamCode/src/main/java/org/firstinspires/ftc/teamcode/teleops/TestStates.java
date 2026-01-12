package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BallDetector;

@TeleOp
public class TestStates extends OpMode {
    BallDetector ballDetector;

    double intakePower = 0;
    boolean intake = false; //false = stop, true = intake
    boolean setPose = false;
    boolean human = false;
    boolean slow = false;
    boolean p2p = false;
    boolean RT = false;
    boolean LT = false;
    boolean moveShot = false;
    boolean disableFlywheel = false;
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        READY,
        SHOOTING,
    } private LaunchState launchState; // Instance

    private enum IntakeState {
        IDLE,
        INTAKING,
        OUTTAKING,
        TRANSFERRING
    } private IntakeState intakeState; // Instance
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(),telemetry);
    public void init(){
        ballDetector = new BallDetector(hardwareMap);
        launchState = LaunchState.IDLE;
        intakeState = IntakeState.IDLE;
//        robot.turret.turnToRobotAngle(0);
//        robot.hood.turnToAngle(Math.toRadians(45));
        joinedTelemetry.update();
    }
//    public void start(){
//        robot.startTeleop();
//    }

    public void updateShooter(Telemetry telemetry, boolean shootRequested, boolean spinup, boolean humanFeed, boolean lock, boolean idle, boolean moveShot, Pose setRobotPose, double flywheelChange, boolean hoodUp, boolean hoodDown, boolean turretLeft, boolean turretRight) {
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
                if (true ){
                    launchState = LaunchState.READY;
                }
                //if idle requested ==> IDLE
                else if (idle){
                    launchState = LaunchState.IDLE;
                }
                break;
            case READY: //ready to shoot
                //if flywheel speed and hood/turret angles are incorrect ==> SPIN_UP
                if (false){
                    launchState = LaunchState.SPIN_UP;
                }
                //if idle requested or (no ball at top AND not transferring) ==> IDLE. idle request should be stronger. but automatic idle should be weaker.
                else if (idle){
                    launchState = LaunchState.IDLE;
                }
                else if (shootRequested){
                    launchState = LaunchState.SHOOTING;
                }
                break;
            case SHOOTING: //ready to shoot
                //if flywheel speed and hood/turret angles are incorrect ==> SPIN_UP
                if (false){
                    launchState = LaunchState.SPIN_UP;
                }
                //if idle requested or (no ball at top AND not transferring) ==> IDLE. idle request should be stronger. but automatic idle should be weaker.
                else if (idle || (!ballDetector.ballPresent(telemetry) && !shootRequested)){
                    launchState = LaunchState.IDLE;
                }
                break;
        }
        telemetry.addData("ball present", ballDetector.ballPresent(telemetry));
        telemetry.addData("shooting", shoot);
        telemetry.addData("launch state", launchState);


    } // -------------------------------END OF UPDATE SHOOTER---------------------------------------

    public void updateIntake(boolean in, boolean out, boolean stopIn, boolean shoot, Telemetry telemetry) {
        if (in) intakePower++;
        // ---------------START OF SHOOTER STATE MANAGER----------------------
        switch (intakeState) {
            case IDLE: // ------------------------------------------------------
                if (shoot) {
                    //TODO: think about removing this. flywheel may slow down after every shot. but this is only starting the shots. so idk
                    if (/*launchState == LaunchState.SHOOTING*/ true /*shooter.isAimed()*/) { //TODO: 0.5s timeout here later
                        intakeState = IntakeState.TRANSFERRING;
                        intakePower = 1;
                    }
                } else if (in) {
intakePower = 1;                    intakeState = IntakeState.INTAKING;
                } else if (out) {
                    intakePower = -1;
                    intakeState = IntakeState.OUTTAKING;
                }
//                if (4) {
//                    intake.outtake();
//                    intakeState = IntakeState.OUTTAKING;
//                }
                break; // END OF IDLE STATE

            case INTAKING:
                if (stopIn){
                    intakePower = 0;
                    intakeState = IntakeState.IDLE;
                }
//                if (ballDetector.ballPresent(telemetry)){ //ball present at top. or 3
//                    intakePower = 0;
//                    intakeState = IntakeState.IDLE;
//                }
//                if (4){
//                    intake.outtake();
//                    intakeState = IntakeState.OUTTAKING;
//                }
                else if (shoot) {
                    //TODO: think about removing this. flywheel may slow down after every shot. but this is only starting the shots. so idk
                    if (/*launchState == LaunchState.SHOOTING*/ true /*shooter.isAimed()*/) { //TODO: 0.5s timeout here later
                        intakeState = IntakeState.TRANSFERRING;
                        intakePower = 1;
                    }
                }
                else if (out) {
                    intakePower = -1;
                    intakeState = IntakeState.OUTTAKING;
                }
                break;
            case OUTTAKING:
                if (!out){
                    intakePower = 0;
                    intakeState = IntakeState.IDLE;
                }
                break;
            case TRANSFERRING: // FEED BALL------------------------------------
                if (!shoot){
                    intakePower = 0;
                    intakeState = IntakeState.IDLE;
                }
                break;
        } // --------------------END OF STATE MANAGER--------------------------------
        telemetry.addData("intake state", intakeState);
        telemetry.addData("intake power", intakePower);
//        telemetry.update();
    } // -------------------------------END OF UPDATE SHOOTER---------------------------------------
//    double turretPos = 0.5;
    public void loop(){
//        if (gamepad1.dpad_up){
//            turretPos += 0.001;
//            robot.turret.turret.setPosition(turretPos);
//        }
//        if (gamepad1.dpad_down){
//            turretPos -= 0.001;
//            robot.turret.turret.setPosition(turretPos);
//        }
        //toggles: LB, x, y
//        if (gamepad1.leftBumperWasPressed()) intake = !intake;
        boolean lb;
        if (gamepad1.left_bumper && !intake) lb = true;
        else lb = false;
        intake = gamepad1.left_bumper;
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) setPose = !setPose;
        if (gamepad2.yWasPressed()) human = !human;
        if (gamepad2.leftStickButtonWasPressed() /*|| gamepad1.leftStickButtonWasPressed()*/) slow = !slow;
//        if (gamepad1.yWasPressed()) p2p = !p2p;
        if (gamepad2.aWasPressed()) moveShot = !moveShot;
//        if (gamepad1.startWasPressed() || gamepad2.startWasPressed()) disableFlywheel = !disableFlywheel;
        RT = gamepad2.right_trigger > 0.1;
        LT = gamepad2.left_trigger > 0.1;
//        robot.updateVoltage(telemetry);
        //final version will be: LB = intake toggle, LT = reverse, RB = shoot request, RT = shoot 1
        ballDetector.update();
        updateIntake(lb, gamepad1.left_trigger > 0.1, lb, gamepad1.right_bumper, joinedTelemetry);
        //final version will be: a = spinup request, b = idle request, x = toggle setPose => shooter lock/manual control, rightstick up/down = flywheel scale, dpad up/down= hood, dpad left/right = turret, gamepad2 y = human feed toggle, start = power flywheel off, gamepad2 bumpers and triggers , gamepad2 a = velocity correction
        updateShooter(joinedTelemetry,gamepad1.right_bumper, gamepad1.a, human, setPose, gamepad1.b, moveShot, null, gamepad1.right_stick_y + gamepad2.right_stick_y, gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed(), gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed(), gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed(), gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed());
        //final: toggle left stick button = slow mode, toggle y = p2p drive but it stops on its own
//        robot.drive2.drawPose(packet);
//        telemetry.addLine(""+robot.turret.turret.getPosition());
//        telemetry.addLine(""+robot.drive2.localizer.getPose().position.x + ", "+robot.drive2.localizer.getPose().position.y + ", "+robot.drive2.localizer.getPose().heading.toDouble());
//        telemetry.addLine(""+robot.hood.HOOD.getPosition());
//        telemetry.addLine(""+robot.flywheel.FLYWHEEL.getMotor().getDirection());
//        telemetry.addLine(""+robot.feeder.BL_FEEDER.getPosition());
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        joinedTelemetry.update();
    }
//    public void stop(){
//        robot.setPose(robot.drive2.localizer.getPose());
//    }
}
