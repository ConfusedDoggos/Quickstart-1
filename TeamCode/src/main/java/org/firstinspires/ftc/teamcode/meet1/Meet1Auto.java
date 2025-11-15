package org.firstinspires.ftc.teamcode.meet1;

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
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Meet 1 Auto", group = "Autonomous")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class Meet1Auto extends LinearOpMode {

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
    public static double PGPIntakeX = 9.0;
    public static double GPPIntakeX = 9.0;
    public static double intakeMaxPower = 0.3;
    public static double shooterSpeedGap = 81;
    public static double shootDistance = 50;

    public static double kp = 0.7;
    public static double ki = 300;
    public static double kd = 0;

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
    public double shooterInput;
    public int launcherState = -1;
    private int ballsLaunched = 0;
    private boolean sequenceFinished = false;
    private boolean launchArtifacts = false;
    private boolean autoInitialized = false;

    //Motor Objects
    private MotorEx intakeMotor, launcher1, launcher2;
    private MotorGroup launcherMotors;

    //Interpolated Look-Up Tables
    public InterpLUT shooterInputLUT = new InterpLUT();
    public InterpLUT shooterVelocityLUT = new InterpLUT();

    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // Panels telemetry

        //Initialize PP Follower
        follower= Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        initMotors(); //Initializes subsystem motors
        buildPaths(); //Initialize all Pedro Paths
        buildTables(); //Initialize lookup tables

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)); //Bulk read to reduce loop time

        while (!opModeIsActive()) {
            if (gamepad1.a) {
                definePoses(true);
                telemetry.addLine("Red alliance Auto");
                telemetry.update();
                autoInitialized = true;
            } else if (gamepad1.b) {
                definePoses(false);
                telemetry.addLine("Blue alliance Auto");
                telemetry.update();
                autoInitialized = true;
            }
            if (!autoInitialized) {
                telemetry.addLine("AUTO NOT INITIALIZED!!!");
            }
        }

        sinceLastBall.reset();
        autoTimer.reset();
        while (opModeIsActive()) {

            hubs.forEach(LynxModule::clearBulkCache); //Bulk reading

            //Update Pedro follower and Panels
            follower.update();
            telemetryM.update();
            currentPose = follower.getPose();


            autoStateMachine(); //Overall state machine function to simplify code

            autoLaunchSequence(); // Ensures that launch sequence will occur when necessary

            updateShooterSpeed(); // Ensures that PID is always updating for motor power

            checkLaunchTime(); // Ensures that launcher will continue even if it doesn't have 3 total balls


//            telemetryM.addData("Elapsed",runtime.toString());
            /*telemetryM.addData("X",currentPose.getX());
            telemetryM.addData("Y",currentPose.getY());
            telemetryM.addData("Heading",currentPose.getHeading());*/
            telemetryM.addData("Launcher State",launcherState);
            telemetryM.addData("Shooter Input",shooterInput);
            telemetryM.addData("Balls launched",ballsLaunched);
            telemetryM.addData("Motor Velocity",launcherMotors.getVelocity());
            telemetryM.addData("Launcher Timer",sinceLastBall);
            telemetryM.update();
        }
    }

    private void autoStateMachine() {
        switch (autoState) {
            case -1:
                break;
            case 0:
                follower.followPath(startToScore); // Move to scoring position
                autoState=1;
                break;
            case 1:
                if (!follower.isBusy()) { //Once move is finished
                    scoreArtifacts(); // Activate scoring procedure
                    autoState=2;
                }
                break;
            case 2:
                if (sequenceFinished) { //Once artifacts are scored
                    sequenceFinished=false; //Reset variable
                    follower.followPath(scoreToPPG); // Move to PPG Intake pos
                    autoState=3;
                }
                break;
            case 3:
                if (!follower.isBusy()) { //Once move is finished
                    intakeArtifacts(); // Activate intake
                    follower.setMaxPower(intakeMaxPower); // Move slower?
                    follower.followPath(PPGToIntake); // Move forwards while intaking
                    autoState=4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                    follower.setMaxPower(1); // Back to normal speed
                    resetMotors(); // Turn off motors
                    rejectArtifacts(); // Turn launcher backwards to get stuck balls hopefully out
                    follower.followPath(PPGIntakeToScore); // Move back to scoring position
                    autoState=5;
                }
                break;
            case 5:
                if (!follower.isBusy()) { //Once move is finished
                    scoreArtifacts(); // Artifact scoring procedure
                    autoState=6;
                }
                break;
            case 6:
                if (sequenceFinished) { //If shooter is finished
                    sequenceFinished = false; // Reset variable
                    follower.followPath(scoreToPGP); // Move to PGP intake position
                    autoState=7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(intakeMaxPower); // Go slower?
                    intakeArtifacts(); // Activate intake
                    follower.followPath(PGPToIntake); // Move forwards while intaking
                    autoState=8;
                }
                break;
            case 8:
                if (!follower.isBusy()) { // Once move is complete
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                    resetMotors(); // Disable intake/launcher
                    rejectArtifacts(); // Run launcher backwards to push out stuck artifacts
                    follower.setMaxPower(1); // Back to max speed
                    follower.followPath(PGPIntakeToScore); // Go back to scoring position
                    autoState=9;
                }
                break;
            case 9:
                if (!follower.isBusy()) { // Once move is complete
                    scoreArtifacts(); // Launcher procedure
                    autoState=10;
                }
                break;
            case 10:
                if (sequenceFinished) { //If shooter is finished
                    sequenceFinished = false; // Reset variable
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
                    intakeArtifacts(); // Activate intake
                    follower.followPath(GPPToIntake); // Move forwards while intaking
                    autoState=12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    resetMotors(); // Disable intake/launcher
                    rejectArtifacts(); // Run launcher backwards to push out stuck artifacts
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
                    scoreArtifacts();
                    autoState = 14;
                }
                break;
            case 14: 
                if (sequenceFinished) {
                    sequenceFinished = false;
                    resetMotors();
                    autoState = 15;
                }
                break;
            case 15:
                follower.followPath(scoreToPark);
                autoState = 16;
                break;
            case 16: //resetting or any other things necessary before autonomous ends
                if (!follower.isBusy()) {
                    resetMotors();
                    autoState=-1;
                }
                break;
        }
    }

    public void intakeArtifacts() { //Just turn on intake, maybe also run launcher in reverse to try to reject balls
        intakeMotor.set(1);
        shooterInput=-0.3;
    }

    public void scoreArtifacts() {
        launchArtifacts = true;
    }

    public void checkLaunchTime() {

    }

    public void autoLaunchSequence() { //Do automatic routine, make a global boolean true once all three balls have been launched or a few seconds have passed
        double motorTargetSpeed = calculateShooterPower(shootDistance);
        double motorTargetVelocity = calculateShooterVelocity(motorTargetSpeed);

        telemetryM.addData("Motor Speed",motorTargetSpeed);
        telemetryM.addData("Motor Velocity Target",motorTargetVelocity);
        if (launcherState == -1 && launchArtifacts) {
            launcherState = 0;
            launchArtifacts = false;
        }
        switch (launcherState) {
            case -1:
                break;
            case 0:
                launcherState = 1;
                break;
            case 1:
                //check if launcher is up to speed
                shooterInput = motorTargetSpeed;
                sinceLastBall.reset();
                if (Math.abs(launcherMotors.getVelocity()) >= motorTargetVelocity-shooterSpeedGap) {
                    intakeMotor.set(intakeLoadSpeed);
                    launcherState = 2;
                }
                break;
            case 2:
                //Check if launcher is below speed by a significant amount
                shooterInput = motorTargetSpeed;
                if (sinceLastBall.seconds() > 1.5) {
                    launcherState=3;
                    ballsLaunched=0;
                    sequenceFinished=true;
                }
                if (Math.abs(launcherMotors.getVelocity()) <= motorTargetVelocity-shooterSpeedGap*1.5) {
                    intakeMotor.set(0);
                    ballsLaunched+=1;
                    launcherState=1;
                    if (ballsLaunched==3) {
                        launcherState = 3;
                        ballsLaunched=0;
                        sequenceFinished = true;
                    }
                }
                break;
            case 3:
                resetMotors();
                launcherState = -1;
                break;
        }
    }

    public void rejectArtifacts() { //Launcher spinning in reverse to try to launch all 3 balls once arriving at destination
        shooterInput = -0.3;
    }

    public void resetMotors() { //Set intake and shooter to 0 when they aren't supposed to be active
        intakeMotor.set(0);
        shooterInput = 0;
    }

    public void updateShooterSpeed() {
        launcherMotors.set(shooterInput);
    }

    private double calculateShooterPower(double distance) {
        if (distance > 46.3 && distance < 75) {
            return shooterInputLUT.get(distance);
        } else {
            return 0;
        }
    }
    private double calculateShooterVelocity(double motorInput) {
        return shooterVelocityLUT.get(motorInput);
    }

    public void buildTables() {
        //create shooting speed lookup table

        //Add values (obtained empirically)
        //Input is distance, output is shooter velocity
        shooterInputLUT.add(46.3,0.55);
        shooterInputLUT.add(52.6,0.605);
        shooterInputLUT.add(61.2,0.65);
        shooterInputLUT.add(76.54,0.67);
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

    public void initMotors() {
        intakeMotor = new MotorEx(hardwareMap,"intakeMotor", Motor.GoBILDA.RPM_435);
        launcher1 = new MotorEx(hardwareMap,"shooterMotor1", Motor.GoBILDA.BARE);
        launcher2 = new MotorEx(hardwareMap,"shooterMotor2", Motor.GoBILDA.BARE);
        launcher2.setInverted(true);
        launcher1.setRunMode(Motor.RunMode.VelocityControl);
        launcher1.setVeloCoefficients(kp,ki,kd);
        launcher2.setRunMode(Motor.RunMode.VelocityControl);
        launcher2.setVeloCoefficients(kp,ki,kd);
        launcherMotors = new MotorGroup(launcher1,launcher2);
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

    public void definePoses(boolean isRed) {
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

        startPose = new Pose(Xorigin+signSwap*32.5, 135.5, Math.toRadians(90)); // Start Pose of our robot.
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

}
