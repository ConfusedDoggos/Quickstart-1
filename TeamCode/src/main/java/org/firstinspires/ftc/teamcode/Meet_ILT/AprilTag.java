package org.firstinspires.ftc.teamcode.Meet_ILT;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

import javax.annotation.processing.Processor;

public class AprilTag {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public Position robotPose = new Position(DistanceUnit.INCH,0,0,0, 0);
    public YawPitchRollAngles robotOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0);


    public void init(HardwareMap hardwareMapRef) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(822.317,822.137,319.495,242.502)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMapRef.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag);
        visionPortal = builder.build();
        PanelsCameraStream.INSTANCE.startStream(visionPortal, 60);
    }

    public Pose3D getTelemetry() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for(AprilTagDetection detection : currentDetections) {
            if(detection.metadata != null) {
                if(detection.metadata.id == 20 || detection.metadata.id == 24) {
                    return detection.robotPose;
                }
                else return new Pose3D(new Position(DistanceUnit.INCH, 0, 0, 0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
            }
            else return new Pose3D(new Position(DistanceUnit.INCH, 0, 0, 0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
        }
        return new Pose3D(new Position(DistanceUnit.INCH, 0, 0, 0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
    }
}
