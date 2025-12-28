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

    public static double linearVelocity = 64.485885;
    public static double lateralVelocity = 51.281485;
    public static double linearDeceleration = -33.753513;
    public static double lateralDeceleration = -53.293454;


    public static FollowerConstants followerConstants = new FollowerConstants();
        //.translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.015, 0))
        //.headingPIDFCoefficients(new PIDFCoefficients(1.25, 0, 0.06, 0.02))
        //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00001,6,0.05))
        //.forwardZeroPowerAcceleration(linearDeceleration)
        //.lateralZeroPowerAcceleration(lateralDeceleration)
        //.mass(9.7)
        //.centripetalScaling(0.002);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            //.xVelocity(linearVelocity)
            //.yVelocity(lateralVelocity)
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
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.1, 0.6);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
