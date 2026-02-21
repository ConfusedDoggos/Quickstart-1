package org.firstinspires.ftc.teamcode.Meet_ILT;

import static org.firstinspires.ftc.teamcode.meet2.Meet2Auto.shooterSpeedGap;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.meet3.Meet3Auto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.FileNotFoundException;
import java.util.List;
import java.util.Objects;


@Autonomous(name = "ILT Auto", group = "Autonomous")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class ILT_Auto extends LinearOpMode {

    //Telemetry Manager

    private boolean isFull = false;


    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //April Tag Variables

    //Pedropathing Variables
    private Pose currentPose;

    private final ElapsedTime launchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private MotorEx fL, fR, bL, bR, launcher1, launcher2, turret, intake;
    private ServoEx hoodServo;
    private DistanceSensor distanceSensor1, distanceSensor2;
    private ColorSensor colorSensor;
    private String DTState="drive", intakeState="idle", turretState="idle", launcherState="idle", hoodState="idle";
    private boolean DTisReady, intakeisReady, turretisReady, launcherisReady;
    private MotorGroup launcher;
    private ElapsedTime teleTimer;
    private MecanumDrive drive;


    //Team Dependents
    public static String team = Meet3Auto.team;
    private double goalID = 20;
    private Pose goalPose;
    private Pose aprilTagPose;
    private Pose poseResetPose;
    private boolean useRealStart = false;
    public static Pose endPose;
    //Lookup Tables
    private InterpLUT velocityLUT = new InterpLUT(), rangeLUT= new InterpLUT(), hoodLUT = new InterpLUT();

    //DT Variables
    private double driveInput, strafeInput, turnInput;
    private double driveAngleDegrees = 0;

    //Launcher Test Variables
    public static double speedOverride = 0;
    public static double angleOverride = 0;

    //Launcher Variables
    public static double kp = 1.5;
    public static double ki = 200;
    public static double kd = 0;
    private double launcherTargetVelocity;
    public static double launcherTestSpeed = 0.65;
    public double odoRange = 0;
    public boolean isIdle = true;

    //Hood Variables
    public double hoodTargetAngle = 40;
    public static double minHoodAngle = 33;
    public static double maxHoodAngle = 57;
    public static double hoodOffset = -1.5;

    //Blocker Variables
    public double closedAngle = 0;
    public double openAngle = 180;

    //Intake Variables
    public static double intakePickupSpeed = 1.0;
    public double transferLoadSpeed = 1.0;
    public static double intakeRejectSpeed = -0.5;

    //Turret Variables
    private PIDFController turretPIDF;
    private PIDController launcherPID;
    public static double turretTolerance = 2;
    public static double tkP = 0.0025;
    public static double tkI = 0;
    public static double tkD = 0.00005;
    public static double tkSCustom = 0.13;
    public static double errorTotal = 30;
    private int turretTargetPos;
    private double angleError;
    private boolean turretManualControl = false;
    private boolean turretAngleLimited = false;
    private double turretDriftOffset = 0;
    private boolean driftAdjustToggle = false;

    private double distance1;
    private double distance2;
    private double colorAlpha;

    private int cyclesSinceUpdate;

    //Sensor Variables
    boolean ballIn1, ballIn2, ballIn3, prevBallIn1, prevBallIn2, prevBallIn3, prev2BallIn1, prev2BallIn2, prev2BallIn3;

    private double voltage;
    private double voltageMultiplier;
    //Timer
    private final ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime shootTimeout = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    //Panels Editable Variables
    public static double intakeMaxPower = 1.0;
    public static double rampMaxPower = 1;
    public static double gateWaitTime = 1.4;
    public static double launchTime15 = 0.4;
    public static double gateWait15 = 1.0;
    public static double intakeMaxPower15 = 0.9;
    public static double rampMaxPower15 = 1.0;
    public static double farLaunchTime = .8;
    public static Pose endPosition = new Pose(0,0,0);

    public double selectedAuto = 15; //default 15 ball



    //PedroPathing PathChains
    public Pose startPose;
    public PathChain ScoreToRamp115twoC;
    public PathChain StartToScore;
    public PathChain ScoreToPGP18;
    public PathChain PGPIntake18;
    public PathChain PGPToScore18;
    public PathChain ScoreToRamp1;
    public PathChain ScoreToRamp2;
    public PathChain IntakeRamp;
    public PathChain RampToScore1;
    public PathChain RampToScore2;
    public PathChain PPGIntake18;
    public PathChain PPGToScore;
    public PathChain ScoreToGPP;
    public PathChain GPPIntake18;
    public PathChain GPPToScore18one;
    public PathChain GPPToScore18two;
    public PathChain Park18;
    public PathChain startToScore15;
    public PathChain scoreToPGP15;
    public PathChain PGPIntake15;
    public PathChain PGPToScore15;
    public PathChain ScoreToRamp115;
    public PathChain ScoreToRamp215;
    public PathChain IntakeRamp15;
    public PathChain RampToScore115;
    public PathChain RampToScore215;
    public PathChain PPGIntake15;
    public PathChain PPGToScore15;
    public PathChain ScoreToGPP15;
    public PathChain GPPIntake15;
    public PathChain GPPToScore15one;
    public PathChain Park15;
    public PathChain startToScore15C;
    public PathChain scoreToPGP15C;
    public PathChain PGPIntake15C;
    public PathChain PGPtoGate15C;
    public PathChain PGPToScore15C;
    public PathChain PGPToScore15C2;
    public PathChain ScoreToRamp115C;
    public PathChain ScoreToRamp215C;
    public PathChain IntakeRamp15C;
    public PathChain RampToScore115C;
    public PathChain RampToScore215C;
    public PathChain PPGIntake15C;
    public PathChain PPGToScore15C;
    public PathChain ScoreToGPP15C;
    public PathChain GPPIntake15C;
    public PathChain GPPToScore15oneC;
    public PathChain Park15C;

    //Changing variables
    public int autoState = 0;


    //Launcher Auto Variables
    public static double launchTime = .85;
    public double preparedLauncherVelocity = 0;


    boolean d1Active = true, cActive = true, d3Active = true;

    //Turret Auto Variables
    public int preparedTargetPos =0;


    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // Panels telemetry
        LoopTimer timer = new LoopTimer();
        initMotors(); //Initializes subsystem motors
        createLUTs(); //Initialize lookup tables
        VoltageSensor voltageSensor;
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        voltage = voltageSensor.getVoltage();

        //voltageMultiplier = ((voltage-12)/1.5);
        voltageMultiplier = 1-((13.0-voltage)/13.0)*2;
        if (voltageMultiplier > 1) {
            voltageMultiplier = 1;
        }
        if (voltageMultiplier < .8) {
            voltageMultiplier = .8;
        }

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)); //Bulk read to reduce loop time

        //Initialize PP Follower
        follower= Constants.createFollower(hardwareMap);
        while (opModeInInit()) {
            telemetry.addLine("Driver: Press left stick for blue team and right stick for red team");
            telemetry.addLine("A for 12 ball, X for 15 ball, B for 18 ball, Y for far zone (9)");
            telemetry.addData("Team Selected:", team);
            telemetry.addData("Auto Selected:",selectedAuto);
            if (selectedAuto == 15.5) telemetry.addLine("THIS IS ALLIANCE COMPATIBLE");
            telemetry.update();
            if (gamepad1.left_stick_button) {
                team = "blue";
            } else if (gamepad1.right_stick_button) {
                team = "red";
            }
            if (gamepad1.a) selectedAuto = 15.5;
            else if (gamepad1.x) selectedAuto = 15;
            else if (gamepad1.b) selectedAuto = 18;
        }
        updateTeamDependents();
        follower.setStartingPose(startPose);


        if (selectedAuto == 15.5) buildPaths15Compatible(); //Initialize all Pedro Paths
        if (selectedAuto == 15) buildPaths15Compatible();
        if (selectedAuto == 18) buildPaths18();


        launchTimer.reset();
        autoTimer.reset();
        while (opModeIsActive()) {

            hubs.forEach(LynxModule::clearBulkCache); //Bulk reading

            //Update Pedro follower and Panels
            follower.update();
            currentPose = follower.getPose();


            if (selectedAuto == 15.5) autoStateMachine15Compatible(); //Overall state machine function to simplify code
            if (selectedAuto == 15) autoStateMachine15();
            if (selectedAuto == 18) autoStateMachine18();

            auto(); //All subsystems

            telemetryUpdate();
            telemetryM.update(telemetry);
            endPosition = currentPose;
        }
        endPosition = currentPose;
    }

    public void telemetryUpdate() {
        telemetryM.addLine("Robot Position");
        telemetryM.addData("X",currentPose.getX());
        telemetryM.addData("Y",currentPose.getY());
        telemetryM.addData("Heading",Math.toDegrees(currentPose.getHeading()));
        telemetryM.addData("Goal Distance",odoRange);
        telemetryM.addLine("Launcher Info");
        telemetryM.addData("Launcher Velocity",launcher.getVelocity());
        telemetryM.addData("Launcher Target",velocityLUT.get(launcherTargetVelocity));
        telemetryM.addData("Launcher Power",launcher.get());
        telemetryM.addLine("Turret Info");
        telemetryM.addData("Turret Target",turretTargetPos);
        telemetryM.addData("Turret Pos",turret.getCurrentPosition());
        telemetryM.addData("Turret Angle",turret.getCurrentPosition() * 360/1076.6);
        telemetryM.addData("Turret Motor Power",turret.get());
        //telemetryM.addData("Distance",testDistanceSensor.getDistance(DistanceUnit.INCH));
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
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setRunMode(Motor.RunMode.RawPower);
        launcher2.setInverted(true);
        launcher1.setVeloCoefficients(kp,ki,kd);
        launcher2.setVeloCoefficients(kp,ki,kd);
        //launcher1.setFeedforwardCoefficients(ks,kv);
        //launcher2.setFeedforwardCoefficients(ks,kv);
        launcher1.setRunMode(Motor.RunMode.VelocityControl);
        launcher2.setRunMode(Motor.RunMode.VelocityControl);
        drive = new MecanumDrive(fL, fR, bL, bR);
        drive.setRightSideInverted(false);
        turretPIDF = new PIDFController(tkP, tkI, tkD, 0);
        turretPIDF.setIntegrationBounds(-errorTotal,errorTotal);
        launcher = new MotorGroup(launcher1, launcher2);
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "firstDistanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "thirdDistanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        hoodServo = new ServoEx(hardwareMap,"hoodServo",0, 300);
        hoodServo.setPwm(new PwmControl.PwmRange(500,2500));
        hoodServo.setInverted(true);
    }

    public void createLUTs() {
        //create shooting speed lookup table

        //Add values (obtained empirically)
        //Input is distance, output is shooter velocity
//        rangeLUT.add(20, 0.43);
//        rangeLUT.add(48, 0.44);
//        rangeLUT.add(55, 0.45);
//        rangeLUT.add(65.7, 0.48);
//        rangeLUT.add(75, 0.5);
//        rangeLUT.add(85, 0.54);
//        rangeLUT.add(100, .57);
//        rangeLUT.add(110, 0.6);
//        rangeLUT.add(115, 0.62);
//        rangeLUT.add(123, 0.69);
//        rangeLUT.add(130, 0.69);
//        rangeLUT.add(140, 0.76);
//        rangeLUT.add(160, 0.81);
        if (speedOverride == 0) {
            rangeLUT.add(47,0.4);
            rangeLUT.add(55,0.41);
            rangeLUT.add(65,0.44);
            rangeLUT.add(75,0.47);
            rangeLUT.add(85,.47);
            rangeLUT.add(95,0.5);
            rangeLUT.add(106,0.52);
            rangeLUT.add(126,.56);
            rangeLUT.add(147,.6);
            rangeLUT.add(160,0.64);
        } else {
            rangeLUT.add(20,speedOverride);
            rangeLUT.add(160,speedOverride);
        }
        rangeLUT.createLUT();

        velocityLUT.add(-1.001,2500);
        velocityLUT.add(-0.8,2000);
        velocityLUT.add(-0.6,1500);
        velocityLUT.add(-.4,1000);
        velocityLUT.add(-.2,500);
        velocityLUT.add(0,0);
        velocityLUT.add(0.2,500);
        velocityLUT.add(0.4,1000);
        velocityLUT.add(0.6,1500);
        velocityLUT.add(0.8,2000);
        velocityLUT.add(1.001,2500);
        velocityLUT.createLUT();

        if (angleOverride == 0) {
            hoodLUT.add(0, 33);
            hoodLUT.add(20, 33);
            hoodLUT.add(47,33);
            hoodLUT.add(55,33);
            hoodLUT.add(65,35);
            hoodLUT.add(75,36);
            hoodLUT.add(85,39);
            hoodLUT.add(95,40);
            hoodLUT.add(106,42);
            hoodLUT.add(126, 45);
            hoodLUT.add(147, 48);
            hoodLUT.add(160,50);
        } else {
            hoodLUT.add(0,angleOverride);
            hoodLUT.add(160,angleOverride);
        }
        hoodLUT.createLUT();
    }

    public void auto() {
        sensorTeleOp();
        DTTeleOp();
        turretTeleOp();
        launcherTeleOp();
        intakeTeleOp();
        hoodTeleOp();
    }

    public void hoodTeleOp() {
        switch (hoodState) {
            case "idle":
                hoodServo.set(hoodToServoAngle(hoodTargetAngle));
                break;
            case "up":
                hoodTargetAngle = maxHoodAngle;
                hoodServo.set(hoodToServoAngle(hoodTargetAngle));
                break;
            case "down":
                hoodTargetAngle = minHoodAngle;
                hoodServo.set(hoodToServoAngle(hoodTargetAngle));
                break;
            case "adjusting":
                hoodTargetAngle = hoodToServoAngle(angleLUT(odoRange));
                hoodServo.set(hoodTargetAngle);
                break;
        }
        telemetryM.addData("Hood Angle",hoodTargetAngle);
        telemetryM.addData("Servo Angle",hoodServo.get());
    }

    public double servoToHoodAngle(double servoInput) {
        return (servoInput) * 13 / 121 + minHoodAngle + hoodOffset;
    }

    public double hoodToServoAngle(double hoodAngle) {
        return (hoodAngle-minHoodAngle - hoodOffset) * 121 / 13;
    }

    public double angleLUT(double range) {
        if (range > 20 && range < 160) return hoodLUT.get(range);
        else return 45;
    }

    public void sensorTeleOp() {
        if(cyclesSinceUpdate == 3) {
            distance1 = distanceSensor1.getDistance(DistanceUnit.INCH);
            prevBallIn1 = ballIn1;
            ballIn1 = distance1 < 4;
        }
        if(cyclesSinceUpdate == 6) {
            colorAlpha = colorSensor.alpha();
            prevBallIn2 = ballIn2;
            ballIn2 = colorAlpha > 70;
        }
        if(cyclesSinceUpdate == 9) {
            distance2 = distanceSensor2.getDistance(DistanceUnit.INCH);
            cyclesSinceUpdate = 0;
            prevBallIn3 = ballIn3;
            ballIn3 = distance2 < 4;
        }
        cyclesSinceUpdate++;

        telemetryM.addData("Distance1", distance1);
        telemetryM.addData("Distance2", distance2);
        telemetryM.addData("Ball in 1?", ballIn1);
        telemetryM.addData("Ball in 2?", ballIn2);
        telemetryM.addData("Ball in 3?", ballIn3);
    }


    public void DTTeleOp() {
        switch (DTState){
            case "idle":
                DTisReady = true;
                break;
            case "drive":
                break;
        }
    }

    public void intakeTeleOp() {
        switch (intakeState){
            case "idle":
                intakeisReady = true;
                intake.stopMotor();
                shootTimeout.reset();
                break;
            case "firing":
//                if (Math.abs(intake.getVelocity()) < 300) intake.set(1.0);
//                else intake.set(transferLoadSpeed);
                intake.set(transferLoadSpeed);
                break;
            case "intaking":
                intake.set(intakePickupSpeed);
                if (ballIn1 && ballIn2 && ballIn3 && prevBallIn1 && prevBallIn2 && prevBallIn3 && prev2BallIn1 && prev2BallIn2 && prev2BallIn3) {
                    intakeState = "idle";
                    //launcherState = "preparing";
                    turretState = "tracking";
                }
                break;
            case "rejecting":
                intake.set(intakeRejectSpeed);
                shootTimeout.reset();
                break;
        }
    }

    public void turretTeleOp() {

        switch (turretState){
            case "idle":
                turretisReady = true;
                break;
            case "tracking": //track goal whenever possible to save on cycle time, or retain a specific angle
                turretTrackProcedure();
                turretisReady = true;
                break;
            case "aiming": //Use apriltag to exactly aim at goal and find range to pass into launcher
                turretisReady = false;
                turretTargetProcedure();
                if (Math.abs(angleError) <= 3) {
                    turretState = "aimed";
                }
                break;
            case "aimed":
                turretTargetProcedure();
                if (Math.abs(angleError) >= 3) {
                    turretState = "aiming";
                }
                break;
            case "preparing":
                turretTargetPos = preparedTargetPos;
                break;
        }
        if (!turretManualControl) turretControlLoop();
    }

    public void launcherTeleOp() {
        double botX = currentPose.getX();
        double botY = currentPose.getY();
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        switch (launcherState){
            case "idle":
                launcherisReady = true;
                isIdle = true;
                launcher.stopMotor();
                odoRange = Math.hypot(goalX-botX,goalY-botY);
                break;
            case "testSpeed":
                launcherTargetVelocity = launcherTestSpeed;
                launcher.set(launcherTargetVelocity);
                break;
            case "beginLaunchSequence":
                launcherSpinUp();
                launchTimer.reset();
                launcher.set(launcherTargetVelocity);
                launcherisReady = false;
                intakeState = "idle";
                launcherState = "aiming";
                turretState = "aiming";
                hoodState = "adjusting";
                break;
            case "aiming":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                intakeState = "idle";
                if (Math.abs(launcher.getVelocity()) > Math.abs(velocityLUT.get(launcherTargetVelocity)) - shooterSpeedGap) {
                    launcherState = "acc_ready";
                }
                break;
            case "acc_ready":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                if (Objects.equals(turretState, "aimed") && follower.getVelocity().getMagnitude() < 6 && Math.abs(follower.getAngularVelocity()) < 0.5) {
                    intakeState = "firing";
                    launcherState = "firing";
                    break;
                }
            case "firing":
                launcherSpinUp();
                //launcher.set(launcherTargetVelocity);
                if (launcher.getVelocity() > velocityLUT.get(launcherTargetVelocity)) {
                    launcher.stopMotor();
                } else launcher.set(1);
                if (Objects.equals(turretState,"aiming") || (follower.getVelocity().getMagnitude() > 6 && Math.abs(follower.getAngularVelocity()) < 0.5)) {
                    intakeState = "idle";
                    launcherState = "acc_ready";
                }
                break;
            case "preparing":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                break;
            case "rejecting":
                launcherTargetVelocity = -.3;
                launcher.set(launcherTargetVelocity);
                launcherisReady=true;
                break;
        }
    }

    public double calculateRangeLUT(double input) {
        if (input < 47) {
            transferLoadSpeed = 0;
            return rangeLUT.get(48);
        } else if (input > 160) {
            transferLoadSpeed = 0;
            return rangeLUT.get(159);
        } else {
            if (70 < input && input < 90) transferLoadSpeed = 0.8 * voltageMultiplier;
            if (90 < input && input < 110) transferLoadSpeed = 0.6 * voltageMultiplier;
            else if (input > 110) transferLoadSpeed = 0.55 * voltageMultiplier;
            else transferLoadSpeed = 1;
            return rangeLUT.get(input);
        }
    }

    public void launcherSpinUp() {
        double botX = currentPose.getX();
        double botY = currentPose.getY();
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        odoRange = Math.hypot(goalX-botX,goalY-botY);
        launcherTargetVelocity = calculateRangeLUT(odoRange);
    }

    public void turretTargetProcedure() {
        turretTrackProcedure();
        if (!turretAngleLimited) angleError = turretTicksToAngle(turretTargetPos-turret.getCurrentPosition());
        else angleError = 100;
    }

    public void turretTrackProcedure() {
        double turretAngle;
        turretAngle = calculateTurretAngle(currentPose.getX(), currentPose.getY(), Math.toDegrees((currentPose.getHeading())));
        turretAngle = turretAngleLimiter(turretAngle);
        //if (follower.getVelocity().getMagnitude() < 2 && Math.abs(follower.getAngularVelocity()) < 0.2)
        turretTargetPos = turretAngleToTicks(turretAngle);
    }

    public double calculateTurretAngle(double botX, double botY, double botHeading) {
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        double targetAngle = Math.toDegrees(Math.atan2(goalY-botY,goalX-botX));
        targetAngle -= botHeading + 180 + turretDriftOffset;
        telemetryM.addData("Target Angle",targetAngle);
        return targetAngle;
    }

    public void turretControlLoop() {
        double output = turretPIDF.calculate(turret.getCurrentPosition(),turretTargetPos);
        double kSFriction;
        if (Math.abs(turretPIDF.getPositionError()) <= turretTolerance) {
            turret.stopMotor();
        }
        else {
            kSFriction = tkSCustom * (Math.abs(turretPIDF.getPositionError()) / turretPIDF.getPositionError());
            output += kSFriction;
            turret.set(output);
        }
    }

    public int turretAngleToTicks(double angle) {
        return (int) (angle * 1076.6 / 360); //978.7 with 435
    }

    public int turretTicksToAngle(double ticks) {
        return (int) (ticks * 360 / 1076.6); //978.7 with 435
    }

    public double turretAngleLimiter(double angleAttempt) {
        double realAngle = angleAttempt;
        if (realAngle > 180) {
            realAngle -= 360;
        } else if (realAngle < -180) {
            realAngle += 360;
        }
        if ((realAngle > 160 && turret.getCurrentPosition() < 0) || (realAngle < -160 && turret.getCurrentPosition() > 0)) {
            turretAngleLimited = true;
            return turretTicksToAngle(turretTargetPos);
        } else turretAngleLimited = false;
        if (realAngle > 160) {
            turretAngleLimited = true;
            realAngle = 160;
        } else if (realAngle < -140) {
            realAngle = -140;
            turretAngleLimited = true;
        }
        return realAngle;
    }

    public void updateTeamDependents() {
        if (Objects.equals(team,"blue")){
            goalID = 20;
            startPose = new Pose(x(30),134,a(270));
            goalPose = new Pose(0,144,Math.toRadians(135));
            poseResetPose = new Pose(114,7,Math.toRadians(90)); //need to find good one
            aprilTagPose = new Pose(15,130,0);
        } else if (Objects.equals(team,"red")) {
            goalID = 24;
            startPose = new Pose(x(30),134,a(270));
            goalPose = new Pose(144,144,Math.toRadians(45));
            poseResetPose = new Pose(30,7,Math.toRadians(90)); //need to find good one
            aprilTagPose = new Pose(129,130,0);
        }
    }

    public double x(double X) { //transform X based on team
        if (Objects.equals(team, "blue")) return X;
        else return 141.5-(X);
    }

    public double a(double angle) { //transform heading based on team
        if (Objects.equals(team, "blue")) return Math.toRadians(angle);
        else return Math.toRadians(-(angle - 90) + 90);
    }

    public void premoveTurret(double x, double y,double heading) {
        double turretAngle;
        turretAngle = calculateTurretAngle(x, y,heading);
        turretAngle = turretAngleLimiter(turretAngle);
        preparedTargetPos = turretAngleToTicks(turretAngle);
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        odoRange = Math.hypot(goalX-x,goalY-y);
        preparedLauncherVelocity = calculateRangeLUT(odoRange);
    }

    public void resetSubsystems() {
        launcherState = "idle";
        intakeState = "idle";
        turretState = "idle";
    }

    public void intakeBalls() {
        launcherState = "rejecting";
        intakeState = "intaking";
    }

    public void launchBalls() {
        launcherState = "beginLaunchSequence";
    }

    Runnable preSpinLauncher = new Runnable() {
        @Override
        public void run() {
            intakeState = "readyToFire";
            launcherState = "preparing";
            turretState = "preparing";
        }
    };

    Runnable stopIntake = new Runnable() {
        @Override
        public void run() {
            intakeState = "idle";
        }
    };

    Runnable rejectIntake = new Runnable() {
        @Override
        public void run() {
            intakeState = "rejecting";
        }
    };


    public void autoStateMachine18() {
        switch (autoState) {
            case -1:
                break;
            case 0:
                transferLoadSpeed = 1.0;
                follower.followPath(StartToScore);
                autoState = 1;
                premoveTurret(x(45),92,Math.toDegrees(a(110)));
                break;
            case 1:
                if (!follower.isBusy()) {
                    launchBalls();
                    waitTimer.reset();
                    autoState = 3;
                }
                break;
            case 2:
                if (waitTimer.seconds() > .1) {
                    autoState = 3;
                }
                break;
            case 3:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(ScoreToPGP18);
                    autoState = 4;
                }
                break;
            case 4:
                if (follower.atParametricEnd() && follower.getHeadingError() < follower.getCurrentPath().getPathEndHeadingConstraint()) {
                    follower.followPath(PGPIntake18,intakeMaxPower,true);
                    intakeBalls();
                    autoState = 5;
                }
                break;
            case 5:
                if (follower.atParametricEnd()) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    follower.followPath(PGPToScore18);
                    premoveTurret(x(48),84,Math.toDegrees(a(223)));
                    autoState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 7;
                }
                break;
            case 7:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(ScoreToRamp1);
                    autoState = 8;
                }
                break;
            case 8:
                if (follower.getTranslationalError().getMagnitude() < 2) {
                    follower.followPath(IntakeRamp,rampMaxPower,true);
                    intakeBalls();
                    launcherState = "rejectFaster";
                    autoState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    autoState = 10;
                    follower.holdPoint(new Pose(x(12.5),59.5,a(140)));
                    waitTimer.reset();
                }
                break;
            case 10:
                if (waitTimer.seconds() > gateWaitTime) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(49),84,Math.toDegrees(a(216)));
                    follower.followPath(RampToScore1);
                    autoState = -5;
                }
                break;
            case -5:
                if (follower.atParametricEnd()) {
                    follower.followPath(RampToScore2);
                    autoState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 12;
                }
                break;
            case 12:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(ScoreToRamp2);
                    autoState = 14;
                }
                break;
            case 14:
                if (follower.getTranslationalError().getMagnitude() < 2) {
                    follower.followPath(IntakeRamp,rampMaxPower,true);
                    intakeBalls();
                    launcherState = "rejectFaster";
                    telemetryM.addData("Drive error",follower.getTranslationalError().getMagnitude());
                    autoState = 15;
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    autoState = 16;
                    waitTimer.reset();
                    follower.holdPoint(new Pose(x(12.5),59.5,a(140)));
                }
                break;
            case 16:
                if (waitTimer.seconds() > gateWaitTime) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(49),84,Math.toDegrees(a(216)));
                    follower.followPath(RampToScore1);
                    autoState = -6;
                }
                break;
            case -6:
                if (follower.getDriveError() < 2) {
                    follower.followPath(RampToScore2);
                    autoState = 17;
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 19;
                }
                break;
            case 18:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    //follower.turnTo(a(180));
                    autoState = 19;
                }
                break;
            case 19:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(PPGIntake18,intakeMaxPower,true);
                    intakeBalls();
                    autoState = 20;
                }
                break;
            case 20:
                if (follower.atParametricEnd()) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(49),84,Math.toDegrees(a(180)));
                    follower.followPath(PPGToScore);
                    autoState = 21;
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 22;
                }
                break;
            case 22:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(ScoreToGPP);
                    autoState = 24;
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    follower.turnTo(a(180));
                    autoState = 24;
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    follower.followPath(GPPIntake18,intakeMaxPower,true);
                    intakeBalls();
                    autoState = 25;
                }
                break;
            case 25:
                if (follower.atParametricEnd()) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(56),80,Math.toDegrees(a(230)));
                    follower.followPath(GPPToScore18one);
                    autoState = -7;
                }
                break;
            case -7:
                if (follower.atParametricEnd()) {
                    follower.followPath(GPPToScore18two);
                    autoState = 26;
                }
                break;
            case 26:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 27;
                }
                break;
            case 27:
                if (launchTimer.seconds() > launchTime) {
                    follower.followPath(Park18);
                    resetSubsystems();
                    autoState = 28;
                }
                break;
            case 28:
                telemetryM.addLine("Autonomous Complete!!! :D");
                endPosition = follower.getPose();
                break;
        }
    }

    public void buildPaths18() { // capital first letter means for the 18 auto
        StartToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(30), 136), new Pose(x(45), 92))
                )
                .setTangentHeadingInterpolation()
                //.setReversed()
                .addTemporalCallback(500, preSpinLauncher)
                .setBrakingStrength(0.68)
                .build();

        ScoreToPGP18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 92), new Pose(x(45), 60))
                )
                .setLinearHeadingInterpolation(a(110), a(180),0.8)
                //.setBrakingStrength(0.6)
                .build();

        PGPIntake18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 60), new Pose(x(22), 60))
                )
                .setTangentHeadingInterpolation()
                //.setBrakingStrength(0.6)
                .build();

        PGPToScore18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(22), 60), new Pose(x(48), 84))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(200,stopIntake)
//                .addTemporalCallback(300, rejectIntake)
//                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .setBrakingStrength(0.66)
                .build();

        ScoreToRamp1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(48), 84), new Pose(x(35), 63))
                )
                .setLinearHeadingInterpolation(a(223), a(140),0.8)
                .build();

        IntakeRamp = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(32), 59.5),
                                new Pose(x(12.5), 59.5)
                        )
                )
                .setConstantHeadingInterpolation(a(140))
                .build();

        RampToScore1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(13.0),59.5), new Pose(x(16),60))
                )
                .setConstantHeadingInterpolation(a(140))
                .setNoDeceleration()
                .addTemporalCallback(200,stopIntake)
//                .addTemporalCallback(300, rejectIntake)
//                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .build();

        RampToScore2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(17), 60), new Pose(x(49), 84))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStrength(0.66)
                .build();

        ScoreToRamp2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(49), 84), new Pose(x(35), 63))
                )
                .setLinearHeadingInterpolation(a(213), a(140),0.8)
                .build();

        //follower.turnTo(a(180));
        PPGIntake18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(49), 84), new Pose(x(20), 84))
                )
                .setTangentHeadingInterpolation()
                .build();

        PPGToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(20), 84), new Pose(x(49), 84))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(200,stopIntake)
//                .addTemporalCallback(300, rejectIntake)
//                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .setBrakingStrength(0.68)
                .build();

        ScoreToGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(49), 84), new Pose(x(48), 65))
                )
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .addPath(
                        new BezierLine(new Pose(x(48),65), new Pose(x(48),39))
                )
                .setLinearHeadingInterpolation(a(270),a(180),.8)
                .build();

        //follower.turnTo(a(180));
        GPPIntake18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(48), 36), new Pose(x(20), 36))
                )
                .setTangentHeadingInterpolation()
                .build();

        GPPToScore18one = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(20), 36), new Pose(x(38),58))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setNoDeceleration()
                .addTemporalCallback(200,stopIntake)
//                .addTemporalCallback(300, rejectIntake)
//                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .build();

        GPPToScore18two = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(38),58),new Pose(x(56),80))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Park18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(56),80), new Pose(x(48),72))
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autoStateMachine15() {
        switch (autoState) {
            case -1:
                break;
            case 0:
                follower.followPath(startToScore15C);
                hoodState = "adjusting";
                autoState = 1;
                premoveTurret(x(45),92,Math.toDegrees(a(289)));
                break;
            case 1:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    autoState = 3;
                }
                break;
            case 3:
                if (shootTimeout.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(scoreToPGP15C);
                    autoState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(PGPIntake15C,intakeMaxPower15,true);
                    intakeBalls();
                    autoState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(PGPToScore15);
                    premoveTurret(x(57),82,Math.toDegrees(a(217)));
                    autoState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    autoState = 7;
                }
                break;
            case 7:
                if (shootTimeout.seconds() > launchTime15) {
                    resetSubsystems();
                    follower.followPath(ScoreToRamp115C);
                    autoState = -8;
                }
                break;
            case -8:
                if (follower.atParametricEnd()) {
                    follower.followPath(ScoreToRamp215C,rampMaxPower15,true);
                    autoState = -9;
                }
                break;
            case -9:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    autoState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy() && waitTimer.seconds() > 0.4) {
                    follower.followPath(IntakeRamp15C);
                    intakeBalls();
                    autoState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    autoState = 10;
                    follower.holdPoint(new Pose(x(13),52,a(140)));
                    waitTimer.reset();
                }
                break;
            case 10:
                if (waitTimer.seconds() > gateWait15 || isFull) {
                    resetSubsystems();
                    premoveTurret(x(55),84,Math.toDegrees(a(217)));
                    follower.followPath(RampToScore115C);
                    autoState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    autoState = 20;
                }
                break;
            case 20:
                if (shootTimeout.seconds() > launchTime15) {
                    resetSubsystems();
                    follower.followPath(PPGIntake15C,intakeMaxPower15,true);
                    intakeBalls();
                    autoState = 21;
                }
                break;
            case 21:
                if (follower.atParametricEnd()) {
                    resetSubsystems();
                    premoveTurret(x(51),84,Math.toDegrees(a(180)));
                    follower.followPath(PPGToScore15C);
                    autoState = 22;
                }
                break;
            case 22:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 1.5) {
                    launchBalls();
                    autoState = 23;
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    follower.followPath(ScoreToGPP);
                    autoState = 24;
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    follower.followPath(GPPIntake15,intakeMaxPower15,true);
                    intakeBalls();
                    autoState = 25;
                }
                break;
            case 25:
                if (!follower.isBusy()) {
                    resetSubsystems();
                    follower.followPath(GPPToScore15oneC);
                    premoveTurret(59,79,a(228));
                    autoState = 26;
                }
            case 26:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 30;
                }
                break;
            case 30:
                if (shootTimeout.seconds() > launchTime15) {
                    follower.followPath(Park15);
                    resetSubsystems();
                    autoState = 31;
                }
                break;
            case 31:
                telemetryM.addLine("15 Autonomous Complete!!! :D");
                endPosition = follower.getPose();
                break;
        }
    }

    public void autoStateMachine15Compatible() {
        switch (autoState) {
            case -1:
                break;
            case 0:
                follower.followPath(startToScore15C);
                hoodState = "adjusting";
                autoState = 1;
                premoveTurret(x(45),92,Math.toDegrees(a(289)));
                break;
            case 1:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    autoState = 3;
                }
                break;
            case 3:
                if (shootTimeout.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(scoreToPGP15C);
                    autoState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(PGPIntake15C,intakeMaxPower15,true);
                    intakeBalls();
                    autoState = -10;
                }
                break;
            case -10:
                if (!follower.isBusy()) {
                    follower.followPath(PGPtoGate15C);
                    resetSubsystems();
                    autoState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(PGPToScore15C);
                    autoState = -100;
                }
                break;
            case -100:
                if (!follower.isBusy()) {
                    premoveTurret(x(57),82,Math.toDegrees(a(225)));
                    follower.followPath(PGPToScore15C2);
                    autoState = 6;
                }
            case 6:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    autoState = 7;
                }
                break;
            case 7:
                if (shootTimeout.seconds() > launchTime15) {
                    resetSubsystems();
                    follower.followPath(ScoreToRamp115C);
                    autoState = -8;
                }
                break;
            case -8:
                if (follower.atParametricEnd()) {
                    follower.followPath(ScoreToRamp215C,rampMaxPower15,true);
                    autoState = -9;
                }
                break;
            case -9:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    autoState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy() && waitTimer.seconds() > 0.4) {
                    follower.followPath(IntakeRamp15C);
                    intakeBalls();
                    autoState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    autoState = 10;
                    follower.holdPoint(new Pose(x(13),52,a(140)));
                    waitTimer.reset();
                }
                break;
            case 10:
                if (waitTimer.seconds() > gateWait15 || isFull) {
                    resetSubsystems();
                    premoveTurret(x(55),84,Math.toDegrees(a(217)));
                    follower.followPath(RampToScore115C);
                    autoState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    autoState = 12;
                }
                break;
            case 12:
                if (shootTimeout.seconds() > launchTime15) {
                    resetSubsystems();
                    follower.followPath(ScoreToRamp115twoC);
                    autoState = 13;
                }
                break;
            case 13:
                if (follower.atParametricEnd()) {
                    follower.followPath(ScoreToRamp215C,rampMaxPower15,true);
                    autoState = 14;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    autoState = 15;
                }
                break;
            case 15:
                if (!follower.isBusy() && waitTimer.seconds() > 0.4) {
                    follower.followPath(IntakeRamp15C);
                    intakeBalls();
                    autoState = 16;
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    autoState = 17;
                    follower.holdPoint(new Pose(x(13),52,a(140)));
                    waitTimer.reset();
                }
                break;
            case 17:
                if (waitTimer.seconds() > gateWait15 || isFull) {
                    resetSubsystems();
                    premoveTurret(x(55),84,Math.toDegrees(a(217)));
                    follower.followPath(RampToScore115C);
                    autoState = 19;
                }
                break;
            case 19:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    autoState = 20;
                }
                break;
            case 20:
                if (shootTimeout.seconds() > launchTime15) {
                    resetSubsystems();
                    follower.followPath(PPGIntake15C,intakeMaxPower15,true);
                    intakeBalls();
                    autoState = 21;
                }
                break;
            case 21:
                if (follower.atParametricEnd()) {
                    resetSubsystems();
                    premoveTurret(x(51),84,Math.toDegrees(a(180)));
                    follower.followPath(PPGToScore15C);
                    autoState = 22;
                }
                break;
            case 22:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 1.5) {
                    launchBalls();
                    autoState = 23;
                }
                break;
            case 23:
                if (shootTimeout.seconds() > launchTime15) {
                    follower.followPath(Park15C);
                    resetSubsystems();
                    autoState = 24;
                }
                break;
            case 24:
                telemetryM.addLine("15 Autonomous Complete!!! :D");
                endPosition = follower.getPose();
                break;
        }
    }

    public void buildPaths15Compatible() {
        startToScore15C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(30), 136), new Pose(x(45), 92))
                )
                .setTangentHeadingInterpolation()
                .addTemporalCallback(300, preSpinLauncher)
                .build();

        scoreToPGP15C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 92), new Pose(x(45), 60))
                )
                .setLinearHeadingInterpolation(a(289), a(180),0.8)
                .build();

        PGPIntake15C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 60), new Pose(x(19), 60))
                )
                .setTangentHeadingInterpolation()
                .build();

        PGPToScore15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(19),60), new Pose(x(57),82))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PGPtoGate15C = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(19),60),
                                new Pose(x(27),70),
                                new Pose(x(17.75),70)
                        )
                )
                .setConstantHeadingInterpolation(a(180))
                .build();

        PGPToScore15C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(17), 70),
                                new Pose(x(57),70)
                                )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PGPToScore15C2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(57),70),
                                new Pose(x(57),82)
                        )
                )
                .setLinearHeadingInterpolation(a(180),a(225),0.8)
                .addTemporalCallback(300, preSpinLauncher)
                .build();

        ScoreToRamp115C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(55), 84), new Pose(x(38), 66))
                )
                .setLinearHeadingInterpolation(a(225), a(135),0.8)
                .setNoDeceleration()
                .build();

        ScoreToRamp115twoC = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(55), 84), new Pose(x(38), 66))
                )
                .setLinearHeadingInterpolation(a(155), a(135),0.8)
                .setNoDeceleration()
                .build();

        ScoreToRamp215C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(32), 59.5),
                                new Pose(x(12.5), 59.5)
                        )
                )
                .setConstantHeadingInterpolation(a(135))
                .build();

        IntakeRamp15C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(12.5), 59.5),
                                new Pose(x(12.5), 52)
                        )
                )
                .setConstantHeadingInterpolation(a(135))
                .build();

        RampToScore115C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(12.5),52), new Pose(x(55),84))
                )
                //.setConstantHeadingInterpolation(a(155))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(300, preSpinLauncher)
                .build();

        PPGIntake15C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(52), 86), new Pose(x(21), 84))
                )
                .setTangentHeadingInterpolation()
                .build();

        PPGToScore15C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(21), 84), new Pose(x(52), 84))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(300, preSpinLauncher)
                .build();

        ScoreToGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(52),84), new Pose(x(52),36))
                )
                .setConstantHeadingInterpolation(a(180))
                .build();

        GPPIntake15C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(52),36), new Pose(x(20),84))
                )
                .setTangentHeadingInterpolation()
                .build();

        GPPToScore15oneC = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(20),84),new Pose(x(59),79))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Park15C = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(52),84), new Pose(x(48),72))
                )
                .setConstantHeadingInterpolation(a(180))
                .build();

        Park15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(59),79), new Pose(x(50),70))
                )
                .setTangentHeadingInterpolation()
                .build();
    }
}

