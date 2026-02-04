package org.firstinspires.ftc.teamcode.prototyping;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

@Configurable
public class Motors {
    MotorEx fL;
    MotorEx fR;
    MotorEx bL;
    MotorEx bR;
    public MecanumDrive drive;

    MotorEx launcher1;
    MotorEx launcher2;
    public MotorGroup launcher;

    public MotorEx intake;
    public MotorEx turret;
    PIDFController turretPIDF;

    private ILT_Teleop mainScript;
    public double odoRange = 0;

    public double launcherTargetVelocity;
    public double transferLoadSpeed = 1.0;

    private InterpLUT rangeLUT = new InterpLUT();
    public InterpLUT velocityLUT = new InterpLUT();

    private boolean turretAngleLimited = false;





    public void init(HardwareMap hardwareMap, ILT_Teleop script) {
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
        launcher1.setVeloCoefficients(Configurables.kp,Configurables.ki,Configurables.kd);
        launcher2.setVeloCoefficients(Configurables.kp,Configurables.ki,Configurables.kd);
        launcher1.setRunMode(Motor.RunMode.VelocityControl);
        launcher2.setRunMode(Motor.RunMode.VelocityControl);

        drive = new MecanumDrive(fL, fR, bL, bR);
        drive.setRightSideInverted(false);

        turretPIDF = new PIDFController(Configurables.tkP, Configurables.tkI, Configurables.tkD, 0);
        turretPIDF.setIntegrationBounds(-Configurables.errorTotal,Configurables.errorTotal);

        launcher = new MotorGroup(launcher1, launcher2);

        mainScript = script;

        rangeLUT = mainScript.rangeLUT;
        velocityLUT = mainScript.velocityLUT;
    }
    public void launcherSpinUp() {
        double botX = mainScript.currentPose.getX();
        double botY = mainScript.currentPose.getY();
        double goalX = mainScript.goalPose.getX();
        double goalY = mainScript.goalPose.getY();
        odoRange = Math.hypot(goalX-botX,goalY-botY);
        launcherTargetVelocity = calculateRangeLUT(odoRange);
    }

    public double calculateRangeLUT(double input) {
        if (input < 20 || input > 160) {
            if (input > 110) transferLoadSpeed = 0.6;
            else transferLoadSpeed = 0.8;
            return Configurables.launcherTestSpeed;
        } else {
            return rangeLUT.get(input);
        }
    }

    public void turretTargetProcedure() {
        turretTrackProcedure();
        if (!turretAngleLimited) mainScript.angleError = turretTicksToAngle(mainScript.turretTargetPos-turret.getCurrentPosition());
        else mainScript.angleError = 100;
    }

    public void turretTrackProcedure() {
        double turretAngle;
        turretAngle = calculateTurretAngle(mainScript.currentPose.getX(), mainScript.currentPose.getY(), Math.toDegrees((mainScript.currentPose.getHeading())));
        turretAngle = turretAngleLimiter(turretAngle);
        //if (follower.getVelocity().getMagnitude() < 2 && Math.abs(follower.getAngularVelocity()) < 0.2)
        mainScript.turretTargetPos = turretAngleToTicks(turretAngle);
    }

    public double calculateTurretAngle(double botX, double botY, double botHeading) {
        double goalX = mainScript.goalPose.getX();
        double goalY = mainScript.goalPose.getY();
        double targetAngle = Math.toDegrees(Math.atan2(goalY-botY,goalX-botX));
        targetAngle -= botHeading + mainScript.turretDriftOffset;
        return targetAngle;
    }

    public void turretControlLoop() {
        double output = turretPIDF.calculate(turret.getCurrentPosition(), mainScript.turretTargetPos);
        double kSFriction;
        if (Math.abs(turretPIDF.getPositionError()) <= Configurables.turretTolerance) {
            turret.stopMotor();
        }
        else {
            kSFriction = Configurables.tkSCustom * (Math.abs(turretPIDF.getPositionError()) / turretPIDF.getPositionError());
            output += kSFriction;
            turret.set(output);
        }
    }

    public int turretAngleToTicks(double angle) {
        return (int) (angle * 978.7 / 360); //978.7 with 435
    }

    public int turretTicksToAngle(double ticks) {
        return (int) (ticks * 360 / 978.7); //978.7 with 435
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
            return turretTicksToAngle(mainScript.turretTargetPos);
        } else turretAngleLimited = false;
        if (realAngle > 135) {
            turretAngleLimited = true;
            realAngle = 135;
        } else if (realAngle < -135) {
            realAngle = -135;
            turretAngleLimited = true;
        }
        return realAngle;
    }
}
