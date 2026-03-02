package org.firstinspires.ftc.teamcode.regionals;

import static org.firstinspires.ftc.teamcode.meet2.Meet2Auto.shooterSpeedGap;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Meet_ILT.ILT_Auto;

import java.util.Objects;

@Configurable
public class StateMachine extends ILT_Auto {
    public static double intakePickupSpeed = 1.0;
    public static double transferLoadSpeed = 1.0;

    public enum LauncherStates {
        IDLE, BEGIN_LAUNCH_SEQUENCE, AIMING, ACC_READY, FIRING, PREPARING, REJECTING, TEST_SPEED
    }
    public enum TurretStates {
        IDLE, TRACKING, AIMING, AIMED, PREPARING
    }
    public enum IntakeStates {
        IDLE, FIRING, INTAKING, REJECTING
    }

    public enum HoodStates {
        IDLE, UP, DOWN, ADJUSTING
    }

    public LauncherStates launcherState = LauncherStates.IDLE;
    public TurretStates turretState = TurretStates.IDLE;
    public IntakeStates intakeState = IntakeStates.IDLE;
    public HoodStates hoodState = HoodStates.IDLE;

    public void StateMachines() {
        IntakeMachine();
        TurretMachine();
        LauncherMachine();
        HoodMachine();
    }

    boolean intakeReady = false;
    boolean turretReady = false;

    private void HoodMachine() {
        switch (hoodState) {
            case IDLE:
                hoodServo.set(hoodToServoAngle(hoodTargetAngle));
                break;
            case UP:
                hoodTargetAngle = maxHoodAngle;
                hoodServo.set(hoodToServoAngle(hoodTargetAngle));
                break;
            case DOWN:
                hoodTargetAngle = minHoodAngle;
                hoodServo.set(hoodToServoAngle(hoodTargetAngle));
                break;
            case ADJUSTING:
                hoodTargetAngle = hoodToServoAngle(angleLUT(odoRange));
                hoodServo.set(hoodTargetAngle);
                break;
        }
    }

    private void IntakeMachine() {
        switch (intakeState) {
            case IDLE:
                intakeReady = true;
                intake.stopMotor();
                shootTimeout.reset();
                break;
            case FIRING:
                intake.set(transferLoadSpeed);
                break;
            case INTAKING:
                intake.set(intakePickupSpeed);
                if (ballIn1 && ballIn2 && ballIn3 && prevBallIn1 && prevBallIn2 && prevBallIn3 && prev2BallIn1 && prev2BallIn2 && prev2BallIn3) {
                    intakeState = IntakeStates.IDLE;
                    turretState = TurretStates.TRACKING;
                }
                break;
            case REJECTING:
                intake.set(intakeRejectSpeed);
                shootTimeout.reset();
                break;
        }
    }

    private void TurretMachine() {
        switch (turretState) {
            case IDLE:
                turretReady = true;
                break;
            case TRACKING:
                turretTrackProcedure();
                turretReady = true;
                break;
            case AIMING:
                turretReady = false;
                turretTargetProcedure();
                if (Math.abs(angleError) <= 3) {
                    turretState = TurretStates.AIMED;
                }
                break;
            case AIMED:
                turretTargetProcedure();
                if (Math.abs(angleError) >= 3) {
                    turretState = TurretStates.AIMING;
                }
                break;
            case PREPARING:
                turretTargetPos = preparedTargetPos;
                break;
        }
        if (!turretManualControl) turretControlLoop();
    }

    private void LauncherMachine() {
        double botX = currentPose.getX();
        double botY = currentPose.getY();
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        switch (launcherState){
            case IDLE:
                launcherisReady = true;
                isIdle = true;
                launcher.stopMotor();
                odoRange = Math.hypot(goalX-botX,goalY-botY);
                break;
            case TEST_SPEED:
                launcherTargetVelocity = launcherTestSpeed;
                launcher.set(launcherTargetVelocity);
                break;
            case BEGIN_LAUNCH_SEQUENCE:
                launcherSpinUp();
                launchTimer.reset();
                launcher.set(launcherTargetVelocity);
                launcherisReady = false;
                intakeState = IntakeStates.IDLE;
                launcherState = LauncherStates.AIMING;
                turretState = TurretStates.AIMING;
                hoodState = HoodStates.ADJUSTING;
                break;
            case AIMING:
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                intakeState = IntakeStates.IDLE;
                if (Math.abs(launcher.getVelocity()) > Math.abs(velocityLUT.get(launcherTargetVelocity)) - shooterSpeedGap) {
                    launcherState = LauncherStates.ACC_READY;
                }
                break;
            case ACC_READY:
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                if (Objects.equals(turretState, TurretStates.AIMED) && follower.getVelocity().getMagnitude() < 6 && Math.abs(follower.getAngularVelocity()) < 0.5) {
                    intakeState = IntakeStates.FIRING;
                    launcherState = LauncherStates.FIRING;
                    break;
                }
            case FIRING:
                launcherSpinUp();
                //launcher.set(launcherTargetVelocity);
                if (launcher.getVelocity() > velocityLUT.get(launcherTargetVelocity)) {
                    launcher.stopMotor();
                } else launcher.set(1);
                if (Objects.equals(turretState,TurretStates.AIMING) || (follower.getVelocity().getMagnitude() > 6 && Math.abs(follower.getAngularVelocity()) < 0.5)) {
                    intakeState = IntakeStates.IDLE;
                    launcherState = LauncherStates.ACC_READY;
                }
                break;
            case PREPARING:
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                break;
            case REJECTING:
                launcherTargetVelocity = -.3;
                launcher.set(launcherTargetVelocity);
                launcherisReady=true;
                break;
        }
    }
}
