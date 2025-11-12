package org.firstinspires.ftc.teamcode.meet1;

import static org.firstinspires.ftc.teamcode.meet1.Meet1Teleop.intakeLoadSpeed;
import static org.firstinspires.ftc.teamcode.meet1.Meet1Teleop.shooterVelocityGap;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

import kotlin.sequences.Sequence;

@Autonomous(name = "Meet 1 Auto", group = "Autonomous")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class Meet1Auto extends LinearOpMode {

    //Telemetry Manager
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //Pedro Variables
    private Pose currentPose;
    private Follower follower;

    //Timer
    private final ElapsedTime runtime = new ElapsedTime();

    //Panels Editable Variables
    public static double intakeDistance = 30;
    public double kp = Meet1Teleop.kp;
    public double ki = Meet1Teleop.ki;
    public double kd = Meet1Teleop.kd;


    //PedroPathing Poses
    private final Pose startPose = new Pose(33, 135, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(48, 96, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose PPGPose = new Pose(46, 83.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PPGPickupPose = new Pose(PPGPose.getX()-intakeDistance, PPGPose.getY(),PPGPose.getHeading());
    private final Pose PGPPose = new Pose(46, 59.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose PGPPickupPose = new Pose(PGPPose.getX()-intakeDistance, PGPPose.getY(),PGPPose.getHeading());
    private final Pose PGPToScoreControlPoint = new Pose(50,60);
    private final Pose parkPose = new Pose(40,70,Math.toRadians(135));

    //PedroPathing PathChains
    private PathChain startToScore;
    private PathChain scoreToPPG;
    private PathChain PPGToIntake;
    private PathChain PPGIntakeToScore;
    private PathChain scoreToPGP;
    private PathChain PGPToIntake;
    private PathChain PGPIntakeToScore;
    private PathChain scoreToPark;

    //Changing variables
    public int autoState = 0;
    public double shooterInput = 0;
    public int launcherState = 0;
    private int ballsLaunched = 0;
    private boolean sequenceFinished = true;
    private boolean launchArtifacts = false;

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

        waitForStart();

        while (opModeIsActive()) {

            hubs.forEach(LynxModule::clearBulkCache); //Bulk reading

            //Update Pedro follower and Panels
            follower.update();
            telemetryM.update();
            currentPose = follower.getPose();


            //Overall state machine function to simplify code
            autoStateMachine();

            updateShooterSpeed(); // Ensures that PID is always updating for motor power

            autoLaunchSequence();

            telemetryM.addData("Elapsed",runtime.toString());
            telemetryM.addData("X",currentPose.getX());
            telemetryM.addData("Y",currentPose.getY());
            telemetryM.addData("Heading",currentPose.getHeading());
            telemetryM.update();
        }
    }

    private void autoStateMachine() {
        switch (autoState) {
            case -1:
                break;
            case 0:
                follower.followPath(startToScore);
                autoState=1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    scoreArtifacts();
                    autoState=2;
                }
                break;
            case 2:
                if (sequenceFinished) { //Once artifacts are scored
                    sequenceFinished=false;
                    follower.followPath(scoreToPPG);
                    autoState=3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intakeArtifacts();
                    follower.setMaxPower(0.5);
                    follower.followPath(PPGToIntake);
                    autoState=4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    resetMotors();
                    rejectArtifacts();
                    follower.followPath(PPGIntakeToScore);
                    autoState=5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    scoreArtifacts();
                    autoState=6;
                }
                break;
            case 6:
                if (sequenceFinished) { //If shooter is finished
                    sequenceFinished = false;
                    follower.followPath(scoreToPGP);
                    autoState=7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    intakeArtifacts();
                    follower.followPath(PGPToIntake);
                    autoState=8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    resetMotors();
                    rejectArtifacts();
                    follower.setMaxPower(1);
                    follower.followPath(PGPIntakeToScore);
                    autoState=9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    scoreArtifacts();
                    autoState=10;
                }
                break;
            case 10:
                if (sequenceFinished) { //If shooter is finished
                    sequenceFinished = false;
                    follower.followPath(scoreToPark);
                    autoState=11;
                }
                break;
            case 11: //resetting or any other things necessary before autonomous ends
                resetMotors();
                autoState=-1;
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

    public void autoLaunchSequence() { //Do automatic routine, make a global boolean true once all three balls have been launched or a few seconds have passed
        double motorTargetSpeed = calculateShooterPower(50);
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
                launcherMotors.set(motorTargetSpeed);
                if (Math.abs(launcherMotors.getVelocity()) >= motorTargetVelocity-shooterVelocityGap) {
                    intakeMotor.set(intakeLoadSpeed);
                    launcherState = 2;
                }
                break;
            case 2:
                //Check if launcher is below speed by a significant amount
                launcherMotors.set(motorTargetSpeed);
                if (Math.abs(launcherMotors.getVelocity()) <= motorTargetVelocity-shooterVelocityGap*1.5) {
                    intakeMotor.set(0);
                    ballsLaunched+=1;
                    launcherState=1;
                }
                if (ballsLaunched==3) {
                    launcherState = 3;
                    ballsLaunched=0;
                    sequenceFinished = true;
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
                .addPath(new BezierCurve(PGPPickupPose,scorePose,PGPToScoreControlPoint))
                .setLinearHeadingInterpolation(PGPPickupPose.getHeading(),scorePose.getHeading())
                .build();
        scoreToPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),parkPose.getHeading())
                .build();
    }

}
