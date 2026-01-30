package org.firstinspires.ftc.teamcode.Meet_ILT;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

public class AprilTag {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public Position robotPose = new Position(DistanceUnit.INCH,0,0,0, 0);
    public YawPitchRollAngles robotOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0);


    public void initilizeTracking() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
    public void updateTelemetry() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for(AprilTagDetection detection : currentDetections) {
            if(detection.metadata != null) {
                if(detection.metadata.name.contains("Obelisk")) {
                    robotPose = detection.robotPose.getPosition();
                    robotOrientation = detection.robotPose.getOrientation();
                }
            }
        }
    }
}
