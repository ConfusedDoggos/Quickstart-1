package org.firstinspires.ftc.teamcode.meet2;

import static org.firstinspires.ftc.teamcode.meet1.Meet1Teleop.intakeLoadSpeed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;

@Autonomous(name = "Meet 2 Auto", group = "Autonomous")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class Meet2Auto extends LinearOpMode {

    //Telemetry Manager
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //Pedro Variables
    private Pose currentPose;

    //Timer
    private final ElapsedTime sinceLastBall = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    //Panels Editable Variables
    public static double PPGIntakeX = 15.5;
    public static double PGPIntakeX = 12;
    public static double GPPIntakeX = 12;
    public static double intakeMaxPower = 0.3;
    public static double shooterSpeedGap = 81;
    public static double shootDistance = 52;
    public static double startingYOffset=0;


    //PedroPathing Poses
    private Pose startPose; // Start Pose of our robot.
    private Pose scorePose; // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Pose PPGPose; // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose PPGPickupPose; // Where to drive to while intaking PPG
    private Pose PGPPose; // Middle (Second Set) of Artifacts from the Spike Mark.
    private Pose PGPPickupPose; // WHere to drive to while intaking PGP
    private Pose PGPToScoreControlPoint; // Control point to define bezier curve
    private Pose GPPPose; // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Pose GPPPickupPose; // Drive forwards to intake artifacts
    private Pose parkPose; // Final position to exit launch zone

    /*
        private final Pose PGBBackPose = new Pose(40, PGPPickupPose.getY(), PGPPickupPose.getHeading());
    */
    //PedroPathing PathChains
    private PathChain startToScore;
    private PathChain scoreToPPG;
    private PathChain PPGToIntake;
    private PathChain PPGIntakeToScore;
    private PathChain scoreToPGP;
    private PathChain PGPToIntake;
    private PathChain PGPIntakeToScore;
    private PathChain scoreToGPP;
    private PathChain GPPToIntake;
    private PathChain GPPIntakeToScore;
    private PathChain scoreToPark;

    //Changing variables
    public int autoState = 0;
        private boolean sequenceFinished = false;
    private boolean autoInitialized = false;

    //April Tag Variables
    private AprilTagProcessor aprilTag;
    private final boolean USE_WEBCAM = true;


    private MotorEx fL, fR, bL, bR, launcher1, launcher2, turret, intake;
    private String DTState="drive", intakeState="idle", turretState="idle", launcherState="idle";
    private boolean DTisReady, intakeisReady, turretisReady, launcherisReady;
    private MotorGroup launcher;
    private ElapsedTime teleTimer;
    private MecanumDrive drive;

    //Team Dependents
    private String team = "blue";
    private double goalID = 20;
    //Lookup Tables
    private InterpLUT velocityLUT = new InterpLUT(), rangeLUT= new InterpLUT();

    //DT Variables
    private double driveInput, strafeInput, turnInput;
    private double driveAngleDegrees = 0;

    //Launcher Variables
    public static double kp = 0.5;
    public static double ki = 200;
    public static double kd = 0;
    public static double ks = 0;
    public static double kv = 0;
    private double launcherTargetVelocity;
    private double ballsLaunched;
    public static double launcherTestSpeed = 0.60;
    public static double spinUpSpeed = 0.6;

    //Intake Variables
    public static double intakePickupSpeed = 1;
    public static double transferLoadSpeed = .8;
    public static double intakeRejectSpeed = -0.5;

    //Turret Variables
    private PIDFController turretPIDF;
    public static double tkP = 3;
    public static double tkI = 10;
    public static double tkD = 0.00001;
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
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // Panels telemetry
        initMotors(); //Initializes subsystem motors
        createLUTs(); //Initialize lookup tables

        //Initialize PP Follower
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


        buildPaths(); //Initialize all Pedro Paths

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)); //Bulk read to reduce loop time

        waitForStart();

        sinceLastBall.reset();
        autoTimer.reset();
        while (opModeIsActive()) {

            hubs.forEach(LynxModule::clearBulkCache); //Bulk reading

            //Update Pedro follower and Panels
            follower.update();
            telemetryM.update();
            currentPose = follower.getPose();


            autoStateMachine(); //Overall state machine function to simplify code

            auto();

//            telemetryM.addData("Elapsed",runtime.toString());
            telemetryM.addData("X",currentPose.getX());
            telemetryM.addData("Y",currentPose.getY());
            telemetryM.addData("Heading",currentPose.getHeading());
            telemetryM.addData("Launcher State",launcherState);
            telemetryM.addData("Balls launched",ballsLaunched);
            telemetryM.addData("Launcher Timer",sinceLastBall);
            telemetryM.update();
        }
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
        launcher = new MotorGroup(launcher1, launcher2);
        launcher.setVeloCoefficients(kp,ki,kd);
        launcher.setFeedforwardCoefficients(ks,kv);
        drive = new MecanumDrive(fL, fR, bL, bR);
        drive.setRightSideInverted(false);
        turretPIDF = new PIDController(tkP, tkI, tkD);
        turretPIDF.setTolerance(8);
        turret.setCachingTolerance(0.08);
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
    public void auto() {
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
        strafeInput = gamepad1.left_stick_x * driveSpeedMulti;
        driveInput = -gamepad1.left_stick_y * driveSpeedMulti;
        turnInput = gamepad1.right_stick_x * driveSpeedMulti;

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
        } else if (gamepad2.right_stick_button){
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
                // launcher.set(launcherTargetVelocity);

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
                if (Math.abs(launcher.getVelocity()) < Math.abs(velocityLUT.get(launcherTargetVelocity)) - 100) {
                    ballsLaunched += 1;
                    launcherState = "aiming";
                    intakeState = "idle";
                    if (ballsLaunched == 3) {
                        ballsLaunched = 0;
                        launcherState = "idle";
                        turretState = "tracking";
                    }
                }
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
        rangeLUT.add(35,.5);
        rangeLUT.add(50,.6);
        rangeLUT.add(76.54,0.67);
        rangeLUT.add(120,.9);

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
                    angleError = detection.ftcPose.yaw;
                    targetAngle = detection.ftcPose.yaw + Math.toDegrees(currentPose.getHeading());
                    targetAngle = turretAngleLimiter(targetAngle);
                    turretTargetPos = turretAngleToTicks(targetAngle);
                }
            }
        } else {
            targetFound = false; // changed for testing, should be false
            angleError = 90; // changed for testing, should be very large
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

    public double turretAngleLimiter(double angleAttempt) {
        double realAngle = angleAttempt;
        if (realAngle > 180) {
            realAngle -= 300;
        } else if (realAngle < -180) {
            realAngle += 300;
        }
        if (realAngle > 100) {
            realAngle = 100;
        } else if (realAngle < -100) {
            realAngle = -100;
        }
        return realAngle;
    }

    public void updateTeamDependents() {

        double signSwap;
        double Xorigin;
        double transform135=0;
        double transform180=0;
        if (Objects.equals(team,"blue")){
            goalID = 20;
            goalOffset = 135;
            signSwap = 1;
            Xorigin = 0;
        } else {
            signSwap = -1;
            Xorigin = 144;
            transform135 = 90;
            transform180=180;
            goalID = 24;
            goalOffset = 45;
        }
        startPose = new Pose(Xorigin+signSwap*32.5, 135.5+startingYOffset, Math.toRadians(90)); // Start Pose of our robot.
        scorePose = new Pose(Xorigin+signSwap*48, 96, Math.toRadians(135-transform135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        PPGPose = new Pose(Xorigin+signSwap*46, 85.5, Math.toRadians(180-transform180)); // Highest (First Set) of Artifacts from the Spike Mark.
        PPGPickupPose = new Pose(Xorigin+signSwap*PPGIntakeX, PPGPose.getY(),PPGPose.getHeading()); // Where to drive to while intaking PPG
        PGPPose = new Pose(Xorigin+signSwap*46, 62.5, Math.toRadians(180-transform180)); // Middle (Second Set) of Artifacts from the Spike Mark.
        PGPPickupPose = new Pose(Xorigin+signSwap*PGPIntakeX, PGPPose.getY(),PGPPose.getHeading()); // WHere to drive to while intaking PGP
        PGPToScoreControlPoint = new Pose(Xorigin+signSwap*50,40); // Control point to define bezier curve
        GPPPose = new Pose(Xorigin+signSwap*100, 37.5, Math.toRadians(180-transform180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
        GPPPickupPose = new Pose(Xorigin+signSwap*GPPIntakeX,GPPPose.getY(),GPPPose.getHeading()); // Drive forwards to intake artifacts
        parkPose = new Pose(Xorigin+signSwap*40,70,Math.toRadians(135-transform135)); // Final position to exit launch zone
    }

    private void autoStateMachine() {
        switch (autoState) {
            case -1:
                break;
            case 0:
                follower.followPath(startToScore); // Move to scoring position
                turretState = "tracking";
                autoState=1;
                break;
            case 1:
                if (!follower.isBusy()) { //Once move is finished
                    launcherState = "beginLaunchSequence"; // Activate scoring procedure
                    autoState=2;
                }
                break;
            case 2:
                if (launcherisReady) { //Once artifacts are scored
                    follower.followPath(scoreToPPG); // Move to PPG Intake pos
                    autoState=3;
                }
                break;
            case 3:
                if (!follower.isBusy()) { //Once move is finished
                    intakeState = "intaking";
                    follower.setMaxPower(intakeMaxPower); // Move slower?
                    follower.followPath(PPGToIntake); // Move forwards while intaking
                    autoState=4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                    follower.setMaxPower(1); // Back to normal speed
                    intakeState = "idle";
                    launcherState = "rejecting";
                    follower.followPath(PPGIntakeToScore); // Move back to scoring position
                    autoState=5;
                }
                break;
            case 5:
                if (!follower.isBusy()) { //Once move is finished
                    launcherState = "beginLaunchSequence"; // Artifact scoring procedure
                    autoState=6;
                }
                break;
            case 6:
                if (launcherisReady) { //If shooter is finished
                    follower.followPath(scoreToPGP); // Move to PGP intake position
                    autoState=7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakeMaxPower); // Go slower?
                    intakeState = "intaking"; // Activate intake
                    follower.followPath(PGPToIntake); // Move forwards while intaking
                    autoState=8;
                }
                break;
            case 8:
                if (!follower.isBusy()) { // Once move is complete
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                    intakeState = "idle"; // Disable intake/launcher
                    launcherState = "rejecting"; // Run launcher backwards to push out stuck artifacts
                    follower.setMaxPower(1); // Back to max speed
                    follower.followPath(PGPIntakeToScore); // Go back to scoring position
                    autoState=9;
                }
                break;
            case 9:
                if (!follower.isBusy()) { // Once move is complete
                    launcherState = "beginLaunchSequence";
                    autoState=10;
                }
                break;
            case 10:
                if (launcherisReady) { //If shooter is finished
                    if (autoTimer.seconds() > 23) {
                        autoState = 15; // park to end auto
                    } else {
                        follower.followPath(scoreToGPP); // Go to parking position
                        autoState=11;
                    }

                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakeMaxPower); // Go slower?
                    intakeState = "intaking"; // Activate intake
                    follower.followPath(GPPToIntake); // Move forwards while intaking
                    autoState=12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    intakeState = "idle"; // Disable intake/launcher
                    launcherState = "rejecting"; // Run launcher backwards to push out stuck artifacts
                    follower.setMaxPower(1); // Back to max speed
                    if (autoTimer.seconds() > 25) {
                        autoState = 16;
                    } else {
                        follower.followPath(GPPIntakeToScore); // Go back to scoring position
                        autoState=13;
                    }
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    launcherState = "beginLaunchSequence";
                    autoState = 14;
                }
                break;
            case 14:
                if (launcherisReady) {
                    autoState = 15;
                }
                break;
            case 15:
                follower.followPath(scoreToPark);
                autoState = 16;
                break;
            case 16: //resetting or any other things necessary before autonomous ends
                if (!follower.isBusy()) {
                    intakeState = "idle";
                    turretState = "idle";
                    launcherState = "idle";
                    autoState=-1;
                }
                break;
        }
    }

    public void buildPaths() {
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();
        scoreToPPG = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,PPGPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),PPGPose.getHeading())
                .setTranslationalConstraint(0.5)
                .build();
        PPGToIntake = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose,PPGPickupPose))
                .setLinearHeadingInterpolation(PPGPose.getHeading(),PPGPickupPose.getHeading())
                .build();
        PPGIntakeToScore = follower.pathBuilder()
                .addPath(new BezierLine(PPGPickupPose,scorePose))
                .setLinearHeadingInterpolation(PPGPickupPose.getHeading(),scorePose.getHeading())
                .build();
        scoreToPGP = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,PGPPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),PGPPose.getHeading())
                .build();
        PGPToIntake = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose,PGPPickupPose))
                .setLinearHeadingInterpolation(PGPPose.getHeading(),PGPPickupPose.getHeading())
                .build();
        PGPIntakeToScore = follower.pathBuilder()
                .addPath(new BezierLine(PGPPickupPose,PGPPose))
                .setLinearHeadingInterpolation(PGPPose.getHeading(),PGPPose.getHeading())
                .addPath(new BezierLine(PGPPose,scorePose))
                .setLinearHeadingInterpolation(PGPPickupPose.getHeading(),scorePose.getHeading())
                .build();
        scoreToGPP = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,GPPPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),GPPPose.getHeading())
                .build();
        GPPToIntake = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose,GPPPickupPose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(),GPPPickupPose.getHeading())
                .build();
        GPPIntakeToScore = follower.pathBuilder()
                .addPath(new BezierLine(GPPPickupPose,GPPPose))
                .setLinearHeadingInterpolation(GPPPickupPose.getHeading(),GPPPose.getHeading())
                .addPath(new BezierLine(GPPPose,scorePose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(),scorePose.getHeading())
                .build();
        scoreToPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),parkPose.getHeading())
                .build();
    }

    /*public void definePoses(String team) {
        double signSwap;
        double Xorigin;
        double transform135=0;
        double transform180=0;
        if (isRed) {
            signSwap = -1;
            Xorigin = 144;
            transform135 = 90;
            transform180=180;
        } else {
            signSwap = 1;
            Xorigin = 0;
        }

        startPose = new Pose(Xorigin+signSwap*32.5, 135.5+startingYOffset, Math.toRadians(90)); // Start Pose of our robot.
        scorePose = new Pose(Xorigin+signSwap*48, 96, Math.toRadians(135-transform135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        PPGPose = new Pose(Xorigin+signSwap*46, 85.5, Math.toRadians(180-transform180)); // Highest (First Set) of Artifacts from the Spike Mark.
        PPGPickupPose = new Pose(Xorigin+signSwap*PPGIntakeX, PPGPose.getY(),PPGPose.getHeading()); // Where to drive to while intaking PPG
        PGPPose = new Pose(Xorigin+signSwap*46, 62.5, Math.toRadians(180-transform180)); // Middle (Second Set) of Artifacts from the Spike Mark.
        PGPPickupPose = new Pose(Xorigin+signSwap*PGPIntakeX, PGPPose.getY(),PGPPose.getHeading()); // WHere to drive to while intaking PGP
        PGPToScoreControlPoint = new Pose(Xorigin+signSwap*50,40); // Control point to define bezier curve
        GPPPose = new Pose(Xorigin+signSwap*100, 37.5, Math.toRadians(180-transform180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
        GPPPickupPose = new Pose(Xorigin+signSwap*GPPIntakeX,GPPPose.getY(),GPPPose.getHeading()); // Drive forwards to intake artifacts
        parkPose = new Pose(Xorigin+signSwap*40,70,Math.toRadians(135-transform135)); // Final position to exit launch zone
    }*/

}
