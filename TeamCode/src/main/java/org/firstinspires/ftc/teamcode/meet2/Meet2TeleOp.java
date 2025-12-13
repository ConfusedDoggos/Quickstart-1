package org.firstinspires.ftc.teamcode.meet2;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.Line;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;
import java.util.Objects;

@TeleOp(name = "Meet 2 Teleop")
@Configurable
public class Meet2TeleOp extends LinearOpMode {


    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //April Tag Variables
    private AprilTagProcessor aprilTag;
    private final boolean USE_WEBCAM = true;

    //Pedropathing Variables
    private Pose currentPose;

    private MotorEx fL, fR, bL, bR, launcher1, launcher2, turret, intake;
    private String DTState="drive", intakeState="idle", turretState="idle", launcherState="idle";
    private boolean DTisReady, intakeisReady, turretisReady, launcherisReady;
    private MotorGroup launcher;
    private ElapsedTime teleTimer;
    private MecanumDrive drive;


    //Team Dependents
    private String team = "blue";
    private double goalID = 20;
    private Pose startPose;

    //Lookup Tables
    private InterpLUT velocityLUT = new InterpLUT(), rangeLUT= new InterpLUT();

    //DT Variables
    private double driveInput, strafeInput, turnInput;
    private double driveAngleDegrees = 0;

    //Launcher Variables
    public static double kp = 1;
    public static double ki = 400;
    public static double kd = 0;
    public static double ks = 0;
    public static double kv = 0;
    private double launcherTargetVelocity;
    private double ballsLaunched;
    public static double launcherTestSpeed = 0.66;
    public static double spinUpSpeed = 0.6;

    //Intake Variables
    public static double intakePickupSpeed = 1;
    public static double transferLoadSpeed = .8;
    public static double intakeRejectSpeed = -0.5;

    //Turret Variables
    private PIDFController turretPIDF;
    public static double tkP = 5;
    public static double tkI = 100;
    public static double tkD = 0.0001;
    private boolean targetFound = false;
    private int turretTargetPos;
    private double angleError;
    private boolean goalVisible = false;
    private double goalOffset;
    private boolean turretAimed = false;
    private boolean turretManualControl = false;

    //The variable to store our instance of the vision portal.

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        LoopTimer timer = new LoopTimer();

        //set the default settings for motors and initializes them (wow)
        initMotors();
        initTrackingSoftware();
        createLUTs();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        teleTimer = new ElapsedTime();
        follower= Constants.createFollower(hardwareMap);
        while (opModeInInit()) {
            telemetry.addLine("Driver: Press left stick for blue team and right stick for red team.");
            telemetry.addData("Team Selected:",team);
            telemetry.update();
            if (gamepad1.left_stick_button) {
                team="blue";
            } else if (gamepad1.right_stick_button) {
                team="red";
            }
        }
        updateTeamDependents();
        follower.setStartingPose(startPose);
        if (opModeIsActive()) {
            teleTimer.reset();
            while (opModeIsActive()) {
                hubs.forEach(LynxModule::clearBulkCache);
                follower.update();
                currentPose = follower.getPose();
                teleOp();

                telemetryUpdate();
                telemetryM.update(telemetry);

            }
        }

    }   // end method runOpMode()



    public void telemetryUpdate() {
        telemetryM.addData("X",currentPose.getX());
        telemetryM.addData("Y",currentPose.getY());
        telemetryM.addData("Heading",currentPose.getHeading());
        telemetryM.addData("Turret Position",turret.getCurrentPosition());
        telemetryM.addData("Turret Angle",turret.getCurrentPosition() * 360/978.7);
        telemetryM.addData("Turret Target",turretTargetPos);
        telemetryM.addData("Turret State",turretState);
        telemetryM.addData("Turret Motor Power",turret.get());
        telemetryM.addData("Launcher Velocity",launcher.getVelocity());
        telemetryM.addData("Launcher power",launcher.get());
    }

    public void initMotors() {
        fL = new MotorEx(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hardwareMap, "bR", Motor.GoBILDA.RPM_312);
        launcher1 = new MotorEx(hardwareMap, "launcherMotor1", Motor.GoBILDA.BARE);
        launcher2 = new MotorEx(hardwareMap, "launcherMotor2", Motor.GoBILDA.BARE);
        intake = new MotorEx(hardwareMap,"intakeMotor",Motor.GoBILDA.BARE);
        turret = new MotorEx(hardwareMap,"turretMotor",Motor.GoBILDA.RPM_435);
        turret.resetEncoder();
        launcher2.setInverted(true);
        launcher1.setVeloCoefficients(kp,ki,kd);
        launcher1.setFeedforwardCoefficients(ks,kv);
        launcher2.setVeloCoefficients(kp,ki,kd);
        launcher2.setFeedforwardCoefficients(ks,kv);
        launcher1.setRunMode(Motor.RunMode.VelocityControl);
        launcher2.setRunMode(Motor.RunMode.VelocityControl);
        drive = new MecanumDrive(fL, fR, bL, bR);
        drive.setRightSideInverted(true);
        turretPIDF = new PIDController(tkP, tkI, tkD);
        turretPIDF.setTolerance(8);
        turret.setCachingTolerance(0.08);
        launcher = new MotorGroup(launcher1, launcher2);
    }

    public void initTrackingSoftware() {
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();



    }
    public void teleOp() {
        DriverInput();
        DTTeleOp();
        turretTeleOp();
        launcherTeleOp();
        intakeTeleOp();
    }

    public void DriverInput() {
        //DT Section
        double driveSpeedMulti;
        driveSpeedMulti = 1;
        if (gamepad1.left_bumper) {
            driveSpeedMulti = 0.5;
        }
        strafeInput = -gamepad1.left_stick_x * driveSpeedMulti;
        turnInput = -gamepad1.left_stick_y * driveSpeedMulti;
        driveInput = gamepad1.right_stick_x * driveSpeedMulti;

        if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
            follower.setPose(new Pose(currentPose.getX(),currentPose.getY(),Math.toRadians(90)));
        }

        //Intake Section
        if (gamepad2.a) {
            intakeState = "intaking";
            launcherState = "rejecting";
        } else if (gamepad2.y) {
            intakeState = "rejecting";
        } else if (gamepad2.b) {
            intakeState = "idle";
            if (Objects.equals(launcherState,"rejecting")) {
                launcherState = "idle";
            }
        } else if (gamepad2.left_bumper) {
            intakeState = "firing";
        }

        //Launcher Section
        if (gamepad2.right_bumper && launcherisReady) {
            launcherState = "beginLaunchSequence";
        } else if (gamepad2.dpad_up) {
            launcherState = "testSpeed";
        } else if (gamepad2.dpad_down) {
            launcherState = "idle";
        } else if (gamepad2.right_stick_button) {
            launcherState = "idle";
            intakeState = "idle";
            turretState = "tracking";
        }
        
        //Turret Section
        if (!targetFound && Math.abs(gamepad2.left_stick_x) > 0.1) {
            turretManualControl = true;
            turret.setRunMode(Motor.RunMode.RawPower);
            if (turret.getCurrentPosition() > -275 && gamepad2.left_stick_x < 0) {
                turret.set(gamepad2.left_stick_x * .4);
            } else if (turret.getCurrentPosition() < 275 && gamepad2.left_stick_x > 0) {
                turret.set(gamepad2.left_stick_x * .4);
            } else {
                turret.set(0);
            }
        } else turretManualControl = false;
        if (gamepad2.dpad_left) { // test
            turretTargetPos = turretAngleToTicks(45);
        } else if (gamepad2.dpad_right) {
            turretTargetPos = turretAngleToTicks(-45);
        }
        if (gamepad2.dpad_up) {
            turretState = "tracking";
        } else if (gamepad2.dpad_down) {
            turretState = "idle";
        }
        driveAngleDegrees = Math.toDegrees(currentPose.getHeading());
    }

    public void DTTeleOp() {

        switch (DTState){
            case "idle":
                DTisReady = true;
                break;
            case "drive":
                if (Math.abs(driveInput) < .1) {
                    driveInput = 0;
                }
                if (Math.abs(strafeInput) < .1) {
                    strafeInput = 0;
                }
                drive.driveRobotCentric(strafeInput,driveInput,turnInput);
        }
    }
    public void intakeTeleOp() {
        switch (intakeState){
            case "idle":
                intakeisReady = true;
                intake.set(0);
                break;
            case "firing":
                intake.set(transferLoadSpeed);
                break;
            case "intaking":
                intake.set(intakePickupSpeed);
                break;
            case "rejecting":
                intake.set(intakeRejectSpeed);
                break;
        }
    }
    public void turretTeleOp() {

        switch (turretState){
            case "idle":
                turretisReady = true;
                targetFound = false;
                break;
            case "tracking": //track goal whenever possible to save on cycle time, or retain a specific angle
                turretTrackProcedure();
                turretisReady = true;
                targetFound = false;
                break;
            case "aiming": //Use apriltag to exactly aim at goal and find range to pass into launcher
                turretisReady = false;
                turretTargetProcedure();
                if (Math.abs(angleError) <= 5) {
                    turretState = "aimed";
                }
                break;
            case "aimed":
                turretTargetProcedure();
                if (Math.abs(angleError) >= 5) {
                    turretState = "aiming";
                }
        }
        if (!turretManualControl) turretControlLoop();
    }
    public void launcherTeleOp() {
        switch (launcherState){
            case "idle":
                launcherisReady = true;
                launcher.stopMotor();
                ballsLaunched = 0;
                break;
            case "testSpeed":
                launcherTargetVelocity = launcherTestSpeed;
                launcher.set(launcherTargetVelocity);
                break;
            case "beginLaunchSequence":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                launcherisReady = false;
                launcherState = "aiming";
                turretState = "aiming";
                break;
            case "aiming":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                if (Math.abs(launcher.getVelocity()) > Math.abs(velocityLUT.get(launcherTargetVelocity)) - 60) {
                    launcherState = "acc_ready";
                }
                break;
            case "acc_ready":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                if (Objects.equals(turretState, "aimed")) {
                    intakeState = "firing";
                    launcherState = "firing";
                    break;
                }
            case "firing":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                /*if (Math.abs(launcher.getVelocity()) < Math.abs(velocityLUT.get(launcherTargetVelocity)) - 100) {
                    ballsLaunched += 1;
                    launcherState = "aiming";
                    intakeState = "idle";
                    if (ballsLaunched == 3) {
                        ballsLaunched = 0;
                        launcherState = "idle";
                        turretState = "tracking";
                    }
                }*/
                break;
            case "rejecting":
                launcherTargetVelocity = -.6;
                launcher.set(launcherTargetVelocity);
                launcherisReady=true;
                break;
        }
    }

    public void createLUTs() {
        //create shooting speed lookup table

        //Add values (obtained empirically)
        //Input is distance, output is shooter velocity
        rangeLUT.add(30,0.6);
        rangeLUT.add(35,.66);

        rangeLUT.createLUT();
        velocityLUT.add(-1,-2300);
        velocityLUT.add(-0.8,-1760);
        velocityLUT.add(-0.7,-1560);
        velocityLUT.add(-0.6,-1320);
        velocityLUT.add(-0.5,-1100);
        velocityLUT.add(-0.4,-880);
        velocityLUT.add(-0.3,-660);
        velocityLUT.add(-0.2,-430);
        velocityLUT.add(-0.1,-210);
        velocityLUT.add(0,0);
        velocityLUT.add(0.1,210);
        velocityLUT.add(0.2,430);
        velocityLUT.add(0.3,660);
        velocityLUT.add(0.4,880);
        velocityLUT.add(0.5,1100);
        velocityLUT.add(0.6,1320);
        velocityLUT.add(0.7,1560);
        velocityLUT.add(0.8,1760);
        velocityLUT.add(1,2300);
        velocityLUT.createLUT();
    }
    
    public void launcherSpinUp() {
        if (!targetFound) {
            launcherTargetVelocity = spinUpSpeed;
        } else {
            if (!aprilTag.getDetections().isEmpty()) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                telemetry.addData("# AprilTags Detected", currentDetections.size());

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == goalID) {
                        launcherTargetVelocity = rangeLUT.get(detection.ftcPose.range);
                        telemetryM.addData("Apriltag Range", detection.ftcPose.range);
                    }
                }
            }
        }
    }

    public void turretTargetProcedure() {
        double targetAngle;
        if (!aprilTag.getDetections().isEmpty()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == goalID) {
                    targetFound = true;
                    angleError = -detection.ftcPose.yaw;
                    telemetryM.addData("angle Error", angleError);
                    targetAngle = detection.ftcPose.yaw + turretTicksToAngle(turret.getCurrentPosition());
                    targetAngle = turretAngleLimiter(targetAngle);
                    turretTargetPos = turretAngleToTicks(targetAngle);
                }
            }
        } else {
            targetFound = true; // changed for testing, should be false
            angleError = 0; // changed for testing, should be very large
            turretTrackProcedure();
            goalVisible = false;
        }

    }
    public void turretTrackProcedure() {
        double turretAngle;
        turretAngle = -driveAngleDegrees + goalOffset;
        turretAngle = turretAngleLimiter(turretAngle);
        turretTargetPos = turretAngleToTicks(turretAngle);
    }
    public void turretControlLoop() {
        double output = turretPIDF.calculate(turret.getCurrentPosition(),turretTargetPos);
        turret.setVelocity(output);
    }

    public int turretAngleToTicks(double angle) {
        return (int) (angle * 978.7 / 360);
    }
    public int turretTicksToAngle(double ticks) {
        return (int) (ticks * 360 / 978.7);
    }

    public double turretAngleLimiter(double angleAttempt) {
        double realAngle = angleAttempt;
        if (realAngle > 180) {
            realAngle -= 360;
        } else if (realAngle < -180) {
            realAngle += 360;
        }
        if (realAngle > 120) {
            realAngle = 120;
        } else if (realAngle < -120) {
            realAngle = -120;
        }
        return realAngle;
    }
    
    public void updateTeamDependents() {
        if (Objects.equals(team,"blue")){
            goalID = 20;
            goalOffset = 135;
            startPose = new Pose(0,0,Math.toRadians(90)); // placeholder
        } else if (Objects.equals(team,"red")) {
            goalID = 24;
            goalOffset = 45;
            startPose = new Pose(0,0,Math.toRadians(90)); // placeholder
        }
    }
}
