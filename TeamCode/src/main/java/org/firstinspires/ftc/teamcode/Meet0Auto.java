package org.firstinspires.ftc.teamcode;
import java.sql.Time;
import java.util.List;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;


@Autonomous(name = "Meet 0 Auto", group = "0: Meet 0")
//@Configurable
@Disabled

public class Meet0Auto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public static int deadzone = 30;
    public static double speed = .5;
    public static double shooterSpeed = 0.75;
    public static double shooterRPM = 450;
    public static double dtSpeedNormal = 1.0;
    public static double dtSpeedSlow = 0.5;
    public static double intakeSpeed = 0.9;
    public static double intakeRejectSpeed = 0.3;

    //ShooterMethod variables
    public static int minimumShooterSpeed = 1300;
    public boolean shootingStarted;
    public boolean shooterFinished;
    public static double intakeLoadSpeed = 1;
    public static double startingVelocity = 0.89;
    public static double tagWeight = 0.015;
    public static double turnSpeed = 0.2;
    public static double lastDitchPercent = 0.05;
    public static double firstMove = 1800;
    public static double secondMove = 1200;
    public static double kp = 1;
    public static double ki = 0;
    public static double kd = 0;
    public static boolean isRedAlliance = true;
    public static double ks = 0;
    public static double kv = 0.1;

    public static double ka = 0;

    public double aprilTagArea;
    public double screenPercentage;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    private MotorEx fL, fR, bL, bR;
    private MotorEx intakeMotor;
    private MotorEx launcher1, launcher2;
    private MotorGroup launcherMotors;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private double dtSpeed;
    private boolean cameraActive;
    private boolean motorToggle = false;
    private int autoSteps = 0;
    private int ballsLaunched;
    private boolean ballLaunchedToggle = false;
    private ElapsedTime autoTimer;

    private double[] cmd_vel = new double[3];

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        LoopTimer timer = new LoopTimer();
        initAprilTag();
        initMotors();
        resetMotors();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));


        waitForStart();
        if (opModeIsActive()) {
            autoTimer.reset();
            while (opModeIsActive()) {
                hubs.forEach(LynxModule::clearBulkCache);
                timer.start();

                switch (autoSteps) {
                    case 0:
                        moveBackwards();
                        telemetryM.addData("Wheel Position", fL.getCurrentPosition());
                        break;
                    case 1:
                        launchRoutine();
                        break;
                    case 2:
                        telemetryM.debug("Autonomous launched");
                        deactivateBallLauncher();
                        intakeMotor.set(0);
                        resetMotors();
                        autoSteps = 3;
                    case 3:
                        parkingSequence();
                        break;
                    case 4:
                        telemetryM.debug("Auto Complete!");
                        break;
                }


                //Telemetry
                List<Double> velocities = launcherMotors.getVelocities();
                telemetryM.addData("Elapsed Time",autoTimer.seconds());
                telemetryM.addData("Left Flywheel Velocity", velocities.get(0));
                telemetryM.addData("Right Flywheel Velocity", velocities.get(1));
                telemetryM.addData("Launcher Velocity", launcherMotors.getVelocity());
                telemetryM.debug("LoopTime:", timer.getMs() / timer.getHz());
                telemetryM.addData("Screen percentage", screenPercentage);
                telemetryM.addData("April tag area (px)", aprilTagArea);
                timer.end();
                telemetryM.update(telemetry);
                sleep(20);
            }
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */

    private void initMotors() {
        fL = new MotorEx(hardwareMap,"fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hardwareMap,"fR",Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hardwareMap,"bL",Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hardwareMap,"bR",Motor.GoBILDA.RPM_312);
        intakeMotor = new MotorEx(hardwareMap,"intakeMotor", Motor.GoBILDA.RPM_435);
        launcher1 = new MotorEx(hardwareMap,"shooterMotor1", Motor.GoBILDA.BARE);
        launcher2 = new MotorEx(hardwareMap,"shooterMotor2", Motor.GoBILDA.BARE);
        launcher2.setInverted(true);
        launcherMotors = new MotorGroup(launcher1,launcher2);
        drive = new MecanumDrive(fL, fR, bL, bR);
        driverOp = new GamepadEx(gamepad1);

        launcherMotors.setRunMode(Motor.RunMode.VelocityControl);
        launcherMotors.setVeloCoefficients(kp, ki, kd);
        launcherMotors.setFeedforwardCoefficients(ks, kv);

    }


    private void moveBackwards() {
        cmd_vel[0] = 0;
        cmd_vel[1] = 0.3;
        cmd_vel[2] = 0;
        drive.driveRobotCentric(
                cmd_vel[0],
                cmd_vel[1],
                cmd_vel[2]
        );
        if (Math.abs(fL.getCurrentPosition()) > firstMove) {
            autoSteps = 1;
            cmd_vel[0] = 0;
            cmd_vel[1] = 0;
            cmd_vel[2] = 0;
            drive.driveRobotCentric(
                    cmd_vel[0],
                    cmd_vel[1],
                    cmd_vel[2]
            );
        }
    }

    private void checkTime() {
        if (autoTimer.seconds() >= 10) {
            if (autoSteps == 1) {
                autoSteps = 2;
            }
        }
    }

    private void parkingSequence() {
        if (isRedAlliance) {
            cmd_vel[0] = -0.3;
        } else if(!isRedAlliance) {
            cmd_vel[0] = 0.3;
        }
        cmd_vel[1] = 0;
        cmd_vel[2] = 0;
        drive.driveRobotCentric(
                cmd_vel[0],
                cmd_vel[1],
                cmd_vel[2]
        );
        if (Math.abs(fL.getCurrentPosition()) > secondMove) {
            autoSteps = 4;
            cmd_vel[0] = 0;
            cmd_vel[1] = 0;
            cmd_vel[2] = 0;
            drive.driveRobotCentric(
                    cmd_vel[0],
                    cmd_vel[1],
                    cmd_vel[2]
            );
        }
    }

    private void launchRoutine() {
        updateMotorVel();
    }

    private void resetMotors() {
        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
        aprilTag.getDetections();

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        telemetry.update();

    }   // end method telemetryAprilTag()

    private void updateMotorVel() {
        if (!aprilTag.getDetections().isEmpty()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetryM.debug("# AprilTags Detected", currentDetections.size());
            Point[] corners = currentDetections.get(0).corners;
            double x_length = corners[0].x - corners[1].x;
            double y_length = corners[0].y - corners[3].y;
            aprilTagArea = x_length * y_length;
            screenPercentage = (aprilTagArea / 2304);
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if(detection.id == 20 || detection.id==24) {
                    if(detection.id == 20) {
                        isRedAlliance = false;
                    } else {
                        isRedAlliance = true;
                    }
                    if (detection.center.x - 320 < -deadzone || detection.center.x - 320 > deadzone) {
                        cmd_vel[0] = -driverOp.getLeftX() * 0.5;
                        cmd_vel[1] = -driverOp.getLeftY() * 0.5;
                        cmd_vel[2] = ((detection.center.x - 320) / 320) * -speed;
                        if (cmd_vel[2] < turnSpeed && cmd_vel[2] > 0) {
                            cmd_vel[2] = turnSpeed;
                        }
                        if (cmd_vel[2] > -turnSpeed && cmd_vel[2] < 0) {
                            cmd_vel[2] = -turnSpeed;
                        }
                    }
                    else {
                        cmd_vel[0] = 0;
                        cmd_vel[1] = 0;
                        cmd_vel[2] = 0;

                        autoLaunchMethod(Math.abs(screenPercentage));
                    }
                }
                else {
                    autoLaunchMethod(lastDitchPercent);
                }
            }   // end for() loop

        }
        else {
            cmd_vel[0] = 0;
            cmd_vel[1] = 0;
            cmd_vel[2] = 0;
        }
        drive.driveRobotCentric(
                cmd_vel[0],
                cmd_vel[1],
                cmd_vel[2]
        );
    }

    private void autoLaunchMethod(double screenPercentage) {
        boolean upToSpeed;
        double motorTargetSpeed = calculateMotorVelocity(screenPercentage);
        checkTime();
        shooterTarget(motorTargetSpeed);
        //begin checking if motor is at target speed
        if (Math.abs(launcherMotors.getVelocity()) >= minimumShooterSpeed) {
            upToSpeed = true;
            if (Math.abs(launcherMotors.getVelocity()) >= minimumShooterSpeed + 60) {
                ballLaunchedToggle = true;
            }
        } else {
            upToSpeed = false;
            if (ballLaunchedToggle) {
                ballsLaunched += 1;
            }
            ballLaunchedToggle = false;
        }
        if (upToSpeed) {
            intakeMotor.set(intakeLoadSpeed);
        } else {
            intakeMotor.set(0);
        } //make intake hold instead of toggle for jeremy
    }

    private void shooterTarget(double motorTarget) {
        if (!motorToggle) {
            launcherMotors.set(motorTarget);
            motorToggle = true;
        }
    }
    private void activateBallLauncher() {
        if (!motorToggle) {
            launcherMotors.set(shooterSpeed);
            motorToggle = true;
        }
    }

    private double calculateMotorVelocity(double tagSize) {
        double motorVelocity;
        motorVelocity = startingVelocity - (tagSize * tagWeight);
        telemetryM.addData("Motor Velocity command",motorVelocity);
        return motorVelocity;
    }

    private void deactivateBallLauncher() {
        launcherMotors.stopMotor();
        motorToggle = false;
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
}   // end class