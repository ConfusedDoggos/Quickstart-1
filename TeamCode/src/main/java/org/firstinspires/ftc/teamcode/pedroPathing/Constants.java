package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
public class Constants {


    public static FollowerConstants followerConstants = new FollowerConstants()
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.25,0,0.02,0.015))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.11))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0,0.1,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009,0,0.00001,.6,0.03))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0.000005,0.6,0.005))
            .forwardZeroPowerAcceleration(-62.8563)
            .lateralZeroPowerAcceleration(-95.8876)
            .mass(9.3);
            //.centripetalScaling(0.005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(74.4975)
            .yVelocity(55.9221)
            .rightFrontMotorName("fR")
            .rightRearMotorName("bR")
            .leftRearMotorName("bL")
            .leftFrontMotorName("fL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.9)
            .strafePodX(.3)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.85,.7);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
