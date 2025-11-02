/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
 * These default parameters are shown as comments in the code below.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Meet 0 Teleop", group = "0: Meet 0")
@Configurable
//@Disabled
public class v1_teleop extends LinearOpMode {


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public static int deadzone = 30;
    public static double turnSpeed = .5;
    public static double shooterSpeed = 0.75;
    public static double shooterRPM = 450;
    public static double dtSpeedNormal = 1.0;
    public static double dtSpeedSlow = 0.5;
    public static double intakeSpeed = 0.9;
    public static double intakeRejectSpeed = 0.3;
    public static double launcherRejectSpeed = 0.7;

    //ShooterMethod variables
    public static int minimumShooterSpeed = 1300;
    public boolean shootingStarted;
    public boolean shooterFinished;
    public static double intakeLoadSpeed = 1;
    public static double startingVelocity = 0.89;
    public static double tagWeight = 0.015;
    public static double minimumTurnSpeed = 0.1;

    public static double kp = 1;
    public static double ki = 0;
    public static double kd = 0;

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
    public GamepadKeys keys = new GamepadKeys();
    public boolean shouldActivateBallLauncher = true;
    private boolean intakeResetToggle = true;

    private double[] cmd_vel = new double[3];

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        LoopTimer timer = new LoopTimer();
        //set the default settings for April Tag detection
        initAprilTag();
        //set the default settings for motors and initializes them (wow)
        initMotors();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                hubs.forEach(LynxModule::clearBulkCache);
                timer.start();

                //Selectively activate/deactivate camera for loop time
                cameraTeleOp();

                //Drive input based on gamepad + apriltag data
                updateMotorVel();
               
                //Check gamepad to activate launcher if commanded
                shooterTeleOp();

                //Update intake power
                intakeTeleOp();

                //Update telemetry for FTCControl Panels
                List<Double> velocities = launcherMotors.getVelocities();
                telemetryM.addData("Left Flywheel Velocity", velocities.get(0));
                telemetryM.addData("Right Flywheel Velocity", velocities.get(1));
                telemetryM.addData("Launcher Velocity",launcherMotors.getVelocity());
                telemetryM.debug("LoopTime:", timer.getMs() / timer.getHz());
                telemetryM.addData("Screen percentage",screenPercentage);
                telemetryM.addData("April tag area (px)",aprilTagArea);

                timer.end();
                //graphM.update();
                telemetryM.update(telemetry);
                // Share the CPU.

                if (cameraActive) {
                    sleep(20);
                }
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



    /**
     * Add telemetry about AprilTag detections.
     */
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
        if (gamepad1.left_trigger > 0.2) {
            dtSpeed = dtSpeedSlow;
        } else {
            dtSpeed = dtSpeedNormal;
        }
        if (gamepad2.left_bumper) {
            autoLaunchMethod(Math.abs(10));
        }
        //checks if it detects any April Tags.
        if (!aprilTag.getDetections().isEmpty() && cameraActive) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            //gets April Tag screen size
            Point[] corners = currentDetections.get(0).corners;
            double x_length = corners[0].x - corners[1].x;
            double y_length = corners[0].y - corners[3].y;
            aprilTagArea = x_length * y_length;
            screenPercentage = (aprilTagArea / 2304);
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if(detection.id == 20 || detection.id==24) {
                    if (detection.center.x - 320 < -deadzone || detection.center.x - 320 > deadzone) {
                        //if it detects april tag, the robot will allow the driver to only control forward and strafing movement.
                        cmd_vel[0] = -driverOp.getLeftX() * 0.5;
                        cmd_vel[1] = -driverOp.getLeftY() * 0.5;
                        //based on april tags position on speed, the robot will set the rotation to move in the opposite direction.
                        cmd_vel[2] = ((detection.center.x - 320) / 320) * -turnSpeed;
                        if (shouldActivateBallLauncher) {
                            activateBallLauncher();
                            shouldActivateBallLauncher = false;
                        }
                        if (cmd_vel[2] < minimumTurnSpeed && cmd_vel[2] > 0) {
                            cmd_vel[2] = minimumTurnSpeed;
                        }
                        if (cmd_vel[2] > -minimumTurnSpeed && cmd_vel[2] < 0) {
                            cmd_vel[2] = -minimumTurnSpeed;
                        }
                        //deactivateBallLauncher();
                    }
                    else {
                        cmd_vel[0] = -driverOp.getLeftX() * 0.5;
                        cmd_vel[1] = -driverOp.getLeftY() * 0.5;
                        cmd_vel[2] = 0;

                        autoLaunchMethod(Math.abs(screenPercentage));
                    }
                }
                else {

                    //deactivateBallLauncher();
                    shouldActivateBallLauncher = true;
                }
            }   // end for() loop

        }
        else {
            shouldActivateBallLauncher = true;
            cmd_vel[0] = -driverOp.getLeftX() * dtSpeed;
            cmd_vel[1] = -driverOp.getLeftY() * dtSpeed;
            cmd_vel[2] = -driverOp.getRightX() * dtSpeed;
        }
        drive.driveRobotCentric(
                cmd_vel[0],
                cmd_vel[1],
                cmd_vel[2]
        );
    }

    private void cameraTeleOp() {
        if (gamepad2.right_trigger > 0.5 && !cameraActive) {
            cameraActive = true;
        } else {
            cameraActive = false;
        }
    }

    private void shooterTeleOp() {
        if (gamepad2.dpad_up) {
            activateBallLauncher();
        } else if (gamepad2.dpad_down) {
            deactivateBallLauncher();
        }
        if (Math.abs(gamepad2.left_stick_x) > 0.1) {
                    launcherMotors.set(gamepad2.left_stick_x);
        }
        if (gamepad2.right_stick_button) {
            deactivateBallLauncher();
            intakeMotor.set(0);
        }
    }

    private void intakeTeleOp() {
        if (gamepad2.a) {
            intakeMotor.set(intakeSpeed);
        } else if (gamepad2.b) {
            intakeMotor.set(0);
        } else if (gamepad2.y) {
            intakeMotor.set(-intakeRejectSpeed);
        }
        if (gamepad2.x && intakeResetToggle) {
            intakeResetToggle = false;
            intakeMotor.set(-intakeRejectSpeed);
            motorToggle = true;
            launcherMotors.set(-launcherRejectSpeed);
            try {
                Thread.sleep(200);
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
            }
            intakeMotor.set(0);
            deactivateBallLauncher();
            intakeResetToggle = true;
        }
    }

    private void autoLaunchMethod(double screenPercentage) {
        boolean upToSpeed;
        double motorTargetSpeed = calculateMotorVelocity(screenPercentage);
        shooterTarget(motorTargetSpeed);
        //begin checking if motor is at target speed
        if (Math.abs(launcherMotors.getVelocity()) >= minimumShooterSpeed) {
            upToSpeed = true;
        } else {
            upToSpeed = false;
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
        /*launcherMotors.set(-0.1 * shooterSpeed);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }*/
        if (!motorToggle) {
            launcherMotors.set(shooterSpeed);
            motorToggle = true;
        }
    }
    private double calculateMotorVelocity(double tagSize) {
        double motorVelocity;
        motorVelocity = startingVelocity - (tagSize * tagWeight);
        telemetryM.addData("Motorvelocity command",motorVelocity);
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