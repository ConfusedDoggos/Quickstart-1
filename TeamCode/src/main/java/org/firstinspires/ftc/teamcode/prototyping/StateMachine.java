package org.firstinspires.ftc.teamcode.prototyping;

import static org.firstinspires.ftc.teamcode.meet2.Meet2Auto.shooterSpeedGap;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

public class StateMachine {
    public String intakeState;
    public String turretState;
    public String launcherState;

    double intakePickupSpeed = 1.0;
    double intakeRejectSpeed = -0.5;

    private final ElapsedTime launchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


    ILT_Teleop mainScript;
    Motors motorHandler;

    public void init(ILT_Teleop main, Motors motor) {
        mainScript = main;
        motorHandler = motor;
    }

    public void updateStates() {
        updateIntake();
        updateTurret();
        updateLauncher();
    }

    public void updateIntake() {
        switch(intakeState) {
            case "idle":
                motorHandler.intake.stopMotor();
                break;
            case "firing":
                if (Math.abs(motorHandler.intake.getVelocity()) < 300) motorHandler.intake.set(1.0);
                else motorHandler.intake.set(motorHandler.transferLoadSpeed);
                break;
            case "intaking":
                motorHandler.intake.set(motorHandler.transferLoadSpeed);
                break;
            case "rejecting":
                motorHandler.intake.set(intakeRejectSpeed);
                break;
        }
    }

    public void updateTurret() {

        switch (turretState){
            case "idle":
                break;
            case "tracking": //track goal whenever possible to save on cycle time, or retain a specific angle
                motorHandler.turretTrackProcedure();
                break;
            case "aiming": //Use apriltag to exactly aim at goal and find range to pass into launcher
                motorHandler.turretTargetProcedure();
                if (Math.abs(mainScript.angleError) <= 3) {
                    turretState = "aimed";
                }
                break;
            case "aimed":
                motorHandler.turretTargetProcedure();
                if (Math.abs(mainScript.angleError) >= 3) {
                    turretState = "aiming";
                }
                break;
            case "preparing":
                break;
        }
        if (!Configurables.turretManualControl) motorHandler.turretControlLoop();
    }
    public void updateLauncher() {
        double botX = mainScript.currentPose.getX();
        double botY = mainScript.currentPose.getY();
        double goalX = mainScript.goalPose.getX();
        double goalY = mainScript.goalPose.getY();
        switch (launcherState){
            case "idle":
                motorHandler.launcher.stopMotor();
                motorHandler.odoRange = Math.hypot(goalX-botX,goalY-botY);
                break;
            case "testSpeed":
                motorHandler.launcherTargetVelocity = Configurables.launcherTestSpeed;
                motorHandler.launcher.set(motorHandler.launcherTargetVelocity);
                break;
            case "beginLaunchSequence":
                motorHandler.launcherSpinUp();
                launchTimer.reset();
                motorHandler.launcher.set(motorHandler.launcherTargetVelocity);
                launcherState = "aiming";
                turretState = "aiming";
                break;
            case "aiming":
                motorHandler.launcherSpinUp();
                motorHandler.launcher.set(motorHandler.launcherTargetVelocity);
                intakeState = "idle";
                if (Math.abs(motorHandler.launcher.getVelocity()) > Math.abs(motorHandler.velocityLUT.get(motorHandler.launcherTargetVelocity)) - shooterSpeedGap) {
                    launcherState = "acc_ready";
                }
                break;
            case "acc_ready":
                motorHandler.launcherSpinUp();
                motorHandler.launcher.set(motorHandler.launcherTargetVelocity);
                if (Objects.equals(turretState, "aimed") && follower.getVelocity().getMagnitude() < 6 && Math.abs(follower.getAngularVelocity()) < 0.5) {
                    intakeState = "firing";
                    launcherState = "firing";
                    break;
                }
            case "firing":
                motorHandler.launcherSpinUp();
                motorHandler.launcher.set(motorHandler.launcherTargetVelocity);
                if (Objects.equals(turretState,"aiming") || (follower.getVelocity().getMagnitude() > 6 && Math.abs(follower.getAngularVelocity()) < 0.5)) {
                    intakeState = "idle";
                    launcherState = "acc_ready";
                }
                break;
            case "preparing":
                motorHandler.launcherSpinUp();
                motorHandler.launcher.set(motorHandler.launcherTargetVelocity);
                break;
            case "rejecting":
                motorHandler.launcherTargetVelocity = -.3;
                motorHandler.launcher.set(motorHandler.launcherTargetVelocity);
                break;
        }
    }
}
