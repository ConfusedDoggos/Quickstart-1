package org.firstinspires.ftc.teamcode.Meet_ILT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public abstract class AprilTag extends LinearOpMode {
    private static AprilTagProcessor aprilTag;
    private static VisionPortal visionPortal;

    public static Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public static Pose3D robotPose;
    public static YawPitchRollAngles robotOrientation;


    public static void initilizeTracking() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                .build();
    }
    public static void updateTelemetry() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for(AprilTagDetection detection : currentDetections) {
            if(detection.metadata != null) {
                if(detection.metadata.name.contains("Obelisk")) {
                    robotPose = detection.robotPose;
                    robotOrientation = detection.robotPose.getOrientation();
                }
            }
        }
    }
}
