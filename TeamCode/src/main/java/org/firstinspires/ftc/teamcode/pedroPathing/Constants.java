package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.65732) // kg mass of robot
            .forwardZeroPowerAcceleration(-35.8478891252221)
            .lateralZeroPowerAcceleration(-75.16567708403743);
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.01, 0))
//            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.01, 0))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0.0,0.00001,0.6,0.0))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.000005,0.6,0.0))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.02, 0, 0.001, 0))
//            .centripetalScaling(0.0005)

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(68.49260417307455)
            .yVelocity(54.68980443759229)
//            .useVoltageCompensation(true)
            .rightFrontMotorName("W_FR") // motor names, keep consistent on configs
            .rightRearMotorName("W_BR")
            .leftRearMotorName("W_BL")
            .leftFrontMotorName("W_FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6) // use inches
            .strafePodX(4.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
