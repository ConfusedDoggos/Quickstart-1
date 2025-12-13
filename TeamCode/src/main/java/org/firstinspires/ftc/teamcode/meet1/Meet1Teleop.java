package org.firstinspires.ftc.teamcode.meet1;
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

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@TeleOp(name = "Meet 1 Teleop")
@Configurable
public class Meet1Teleop extends LinearOpMode {


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public static int deadzone = 30;
    public static double turnSpeed = .5;
    public static double shooterSpeed = 0.75;
    public static double dtSpeedNormal = 1.0;
    public static double dtSpeedSlow = 0.5;
    public static double intakeSpeed = 0.9;
    public static double intakeRejectSpeed = -0.3;
    public static double launcherRejectSpeed = -0.4;

    //ShooterMethod variables
    public boolean shootingStarted;
    public boolean shooterFinished;
    public static double intakeLoadSpeed = 1;
    public static double minimumTurnSpeed = 0.1;
    public static double spinUpSpeed = 0.8;

    public static double kp = 0.7;
    public static double ki = 300;
    public static double kd = 0;

    public static double ks = 0;
    public static double kv = 0;

    public static double ka = 0;

    public static int shooterVelocityGap = 100;
    public static double testSpeed = 0.5;

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
    private boolean intakeResetToggle = true;
    public InterpLUT shooterInputLUT = new InterpLUT();
    public InterpLUT shooterVelocityLUT = new InterpLUT();
    private ElapsedTime teleTimer;
    private boolean blindFire = false;

    private int ballsLaunched = 0;
    private int launcherState = -1;
    private boolean goalInSight = false;
    private double shooterInputSpeed;


    private double goalRange;

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

        //Create the Interpolated Lookup Table
        createInterpLookUpTables();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        teleTimer = new ElapsedTime();

        waitForStart();
        if (opModeIsActive()) {
            teleTimer.reset();
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

                //Activates automatic launching sequence when necessary
                autoLaunchMethod();

                updateShooterSpeed();


                //Update telemetry for FTCControl Panels
                List<Double> velocities = launcherMotors.getVelocities();
                telemetryM.addData("Left Flywheel Velocity", velocities.get(0));
                telemetryM.addData("Right Flywheel Velocity", velocities.get(1));
                telemetryM.addData("Launcher Velocity",launcherMotors.getVelocity());
                telemetryM.debug("LoopTime:", timer.getMs() / timer.getHz());
                telemetryM.addData("Distance From Goal", goalRange);
                telemetryM.addData("MotorVelocityAttempt",launcherMotors.get());
                telemetryM.addData("Balls Launched",ballsLaunched);
                telemetryM.addData("Launcher State",launcherState);
                timer.end();
                //graphM.update();
                telemetryM.update(telemetry);
                // Share the CPU.

            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()


    private void initMotors() {
        fL = new MotorEx(hardwareMap,"fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hardwareMap,"fR",Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hardwareMap,"bL",Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hardwareMap,"bR",Motor.GoBILDA.RPM_312);
        intakeMotor = new MotorEx(hardwareMap,"intakeMotor", Motor.GoBILDA.RPM_435);
        launcher1 = new MotorEx(hardwareMap,"shooterMotor1", Motor.GoBILDA.BARE);
        launcher2 = new MotorEx(hardwareMap,"shooterMotor2", Motor.GoBILDA.BARE);
        launcher2.setInverted(true);

        drive = new MecanumDrive(fL, fR, bL, bR);
        driverOp = new GamepadEx(gamepad1);

        launcher1.setRunMode(Motor.RunMode.VelocityControl);
        launcher1.setVeloCoefficients(kp,ki,kd);
        //launcher1.setFeedforwardCoefficients(ks,kv);
        launcher2.setRunMode(Motor.RunMode.VelocityControl);
        launcher2.setVeloCoefficients(kp,ki,kd);
        //launcher2.setFeedforwardCoefficients(ks,kv);
        launcherMotors = new MotorGroup(launcher1,launcher2);
    }

    private void createInterpLookUpTables() {
        //create shooting speed lookup table

        //Add values (obtained empirically)
        //Input is distance, output is shooter velocity
        shooterInputLUT.add(30,0.5);
        shooterInputLUT.add(39,0.53);
        shooterInputLUT.add(46.3,0.55);
        shooterInputLUT.add(52.6,0.605);
        shooterInputLUT.add(61.2,0.63);
        shooterInputLUT.add(76.54,0.6);
        //shooterInputLUT.add(104,0.85);

        /*
        shooterInputLUT.add(95,);
        shooterInputLUT.add(100,);
        shooterInputLUT.add(105,);
        shooterInputLUT.add(110,);
         */

        shooterInputLUT.createLUT();
        //May need to create separate LUT for far zone, unsure if will be necessary or not.
        shooterVelocityLUT.add(-1,-1900);
        shooterVelocityLUT.add(-0.9,-1900);
        shooterVelocityLUT.add(-0.8,-1760);
        shooterVelocityLUT.add(-0.7,-1560);
        shooterVelocityLUT.add(-0.6,-1320);
        shooterVelocityLUT.add(-0.5,-1100);
        shooterVelocityLUT.add(-0.4,-880);
        shooterVelocityLUT.add(-0.3,-660);
        shooterVelocityLUT.add(-0.2,-430);
        shooterVelocityLUT.add(-0.1,-210);
        shooterVelocityLUT.add(0,0);
        shooterVelocityLUT.add(0.1,210);
        shooterVelocityLUT.add(0.2,430);
        shooterVelocityLUT.add(0.3,660);
        shooterVelocityLUT.add(0.4,880);
        shooterVelocityLUT.add(0.5,1100);
        shooterVelocityLUT.add(0.6,1320);
        shooterVelocityLUT.add(0.7,1560);
        shooterVelocityLUT.add(0.8,1760);
        shooterVelocityLUT.add(0.9,1900);
        shooterVelocityLUT.add(1,1900);
        shooterVelocityLUT.createLUT();
    }

    private double calculateShooterPower(double distance) {
        if (distance > 30.5 && distance < 75) {
            return shooterInputLUT.get(distance);
        } else {
            return 0;
        }
    }

    private double calculateShooterVelocity(double motorInput) {
        return shooterVelocityLUT.get(motorInput);
    }



//    private void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//        aprilTag.getDetections();
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
//        telemetry.update();
//
//    }   // end method telemetryAprilTag()


    private void updateShooterSpeed() { // Always updating PID
        if (shooterInputSpeed != 0) {
            launcherMotors.set(shooterInputSpeed);
        } else {
            launcherMotors.stopMotor();
        }
    }

    private void updateMotorVel() {
        double trueDeadzone;
        if (gamepad1.left_trigger > 0.2) {
            dtSpeed = dtSpeedSlow;
        } else {
            dtSpeed = dtSpeedNormal;
        }
        //checks if it detects any April Tags.
        if (!aprilTag.getDetections().isEmpty() && cameraActive) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if(detection.id == 20 || detection.id==24) {
                    goalRange = detection.ftcPose.range;
                    //Finds range, then adjusts deadzone size based on distance
                    if (goalRange > 60) {
                        trueDeadzone = 15;
                    } else{
                        trueDeadzone = 30;
                    }
                    if (detection.center.x - 320 < -trueDeadzone || detection.center.x - 320 > trueDeadzone) {
                        //if it detects april tag, the robot will allow the driver to only control forward and strafing movement.
                        cmd_vel[0] = -driverOp.getLeftX() * 0.5;
                        cmd_vel[1] = -driverOp.getLeftY() * 0.5;
                        //based on april tags position on speed, the robot will set the rotation to move in the opposite direction.
                        cmd_vel[2] = ((detection.center.x - 320) / 320) * -turnSpeed;
                        if (cmd_vel[2] < minimumTurnSpeed && cmd_vel[2] > 0) {
                            cmd_vel[2] = minimumTurnSpeed;
                        }
                        if (cmd_vel[2] > -minimumTurnSpeed && cmd_vel[2] < 0) {
                            cmd_vel[2] = -minimumTurnSpeed;
                        }
                        goalInSight = false;
                    }
                    else {

                        cmd_vel[0] = -driverOp.getLeftX() * 0.5;
                        cmd_vel[1] = -driverOp.getLeftY() * 0.5;
                        cmd_vel[2] = 0;
                        goalInSight = true;
                    }
                }
                else {
                    //only if a non-goal is recognized (later apply for team-based opmodes?
                    goalInSight = false;
                }
            }   // end for() loop

        }
        else {
            cmd_vel[0] = -driverOp.getLeftX() * dtSpeed;
            cmd_vel[1] = -driverOp.getLeftY() * dtSpeed;
            cmd_vel[2] = -driverOp.getRightX() * dtSpeed;
            goalInSight = false;
        }
        drive.driveRobotCentric(
                cmd_vel[0],
                cmd_vel[1],
                cmd_vel[2]
        );
    }

    private void cameraTeleOp() {
        if (gamepad2.right_trigger > 0.5) {
            cameraActive = true;
            visionPortal.setProcessorEnabled(aprilTag, true);
        } else {
            cameraActive = false;
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
    }

    private void shooterTeleOp() {
        if (Math.abs(gamepad2.left_stick_x) > 0.1) {
            shooterInputSpeed = gamepad2.left_stick_x;
        }
        if (gamepad2.right_stick_button) {
            deactivateBallLauncher();
            intakeMotor.set(0);
            launcherState = -1;
            blindFire=false;
        }
        if (gamepad2.dpad_up) {
            if (launcherState == -1) {
                launcherState = 0;
                goalRange = 50;
            }
        }
        if (gamepad2.dpad_left) {
            blindFire = true;
            goalRange=50;
        }
    }

    private void intakeTeleOp() {
        if (gamepad2.a) {
            intakeMotor.set(intakeSpeed);
        } else if (gamepad2.b) {
            intakeMotor.set(0);
        } else if (gamepad2.y) {
            intakeMotor.set(intakeRejectSpeed);
        }
        if (gamepad2.x && intakeResetToggle) {
            intakeResetToggle = false;
            intakeMotor.set(intakeRejectSpeed);
            shooterInputSpeed = launcherRejectSpeed;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            intakeMotor.set(0);
            intakeResetToggle = true;

        }



    }

    private void autoLaunchMethod() {
        double motorTargetSpeed = calculateShooterPower(goalRange);
        double motorTargetVelocity = calculateShooterVelocity(motorTargetSpeed);

        telemetryM.addData("Motor Speed",motorTargetSpeed);
        telemetryM.addData("Motor Velocity Target",motorTargetVelocity);
        if (goalInSight && launcherState == -1 && gamepad2.right_trigger > 0.5) {
            launcherState = 0;
        }
        if (blindFire && launcherState == -1) {
            launcherState=0;
        }
        switch (launcherState) {
            case -1:
                break;
            case 0:
                launcherState = 1;
                break;
            case 1:
                //check if launcher is up to speed
                shooterInputSpeed = motorTargetSpeed;
                if (Math.abs(launcherMotors.getVelocity()) >= motorTargetVelocity-shooterVelocityGap) {
                    intakeMotor.set(intakeLoadSpeed);
                    launcherState = 2;
                }
                break;
            case 2:
                //Check if launcher is below speed by a significant amount
                shooterInputSpeed = motorTargetSpeed;
                if (Math.abs(launcherMotors.getVelocity()) <= motorTargetVelocity-shooterVelocityGap*1.5) {
                    intakeMotor.set(0);
                    ballsLaunched+=1;
                    launcherState=1;
                }
                if (ballsLaunched==3) {
                    launcherState = 3;
                    ballsLaunched=0;
                    blindFire=false;
                }
                break;
            case 3:
                deactivateBallLauncher();
                launcherState = -1;
                break;
        }
    }
    private void shooterTarget(double motorTarget) {
        launcherMotors.set(motorTarget);
    }
    private void activateBallLauncher() {
        launcherMotors.set(shooterSpeed);
    }
    private void deactivateBallLauncher() {
        shooterInputSpeed = 0;
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