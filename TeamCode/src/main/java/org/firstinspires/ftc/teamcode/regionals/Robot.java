package org.firstinspires.ftc.teamcode.regionals;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class Robot {
    private HardwareMap hardwareMap;

    public MotorEx fL, fR, bL, bR, launcher1, launcher2, turret, intake;
    public MecanumDrive drive;
    public MotorGroup launcher;

    public PIDFController turretPIDF;

    public ServoEx hoodServo;

    public static double kp = 1.5;
    public static double ki = 200;
    public static double kd = 0;
    public static double tkP = 0.0025;
    public static double tkI = 0;
    public static double tkD = 0.00005;

    public DistanceSensor distanceSensor1;
    public DistanceSensor distanceSensor2;
    public ColorSensor colorSensor;

    public static double errorTotal = 30;

    public InterpLUT hoodLUT;
    public InterpLUT velocityLUT;
    public InterpLUT rangeLUT;

    public Robot() {
        //constructor thingy, we can set default stuff here (NOTHING HARDWARE RELATED), can take an input btw
    }

    public void init(HardwareMap inHardwareMap) {
        hardwareMap = inHardwareMap;
        initMotors();
        initSensors();
        initLUTs();
    }

    private void initMotors() {
        fL = new MotorEx(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hardwareMap, "bR", Motor.GoBILDA.RPM_312);
        launcher1 = new MotorEx(hardwareMap, "launcherMotor1", Motor.GoBILDA.BARE);
        launcher2 = new MotorEx(hardwareMap, "launcherMotor2", Motor.GoBILDA.BARE);
        intake = new MotorEx(hardwareMap,"intakeMotor",Motor.GoBILDA.BARE);
        turret = new MotorEx(hardwareMap,"turretMotor",Motor.GoBILDA.RPM_435);

        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setRunMode(Motor.RunMode.RawPower);
        launcher2.setInverted(true);
        launcher1.setVeloCoefficients(kp,ki,kd);
        launcher2.setVeloCoefficients(kp,ki,kd);

        launcher1.setRunMode(Motor.RunMode.VelocityControl);
        launcher2.setRunMode(Motor.RunMode.VelocityControl);
        drive = new MecanumDrive(fL, fR, bL, bR);
        drive.setRightSideInverted(false);
        turretPIDF = new PIDFController(tkP, tkI, tkD, 0);
        turretPIDF.setIntegrationBounds(-errorTotal,errorTotal);

        launcher = new MotorGroup(launcher1, launcher2);

        hoodServo = new ServoEx(hardwareMap,"hoodServo",0, 300);
        hoodServo.setPwm(new PwmControl.PwmRange(500,2500));
        hoodServo.setInverted(true);
    }
    private void initSensors() {
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "firstDistanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "thirdDistanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    private void initLUTs() {
        double angleOverride = 0;
        double speedOverride = 0;

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
    
}
