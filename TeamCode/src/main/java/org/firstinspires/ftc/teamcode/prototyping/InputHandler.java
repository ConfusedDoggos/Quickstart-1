package org.firstinspires.ftc.teamcode.prototyping;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;

public class InputHandler {
    double driveInput;
    double strafeInput;
    double turnInput;

    private boolean driftAdjustToggle = false;

    private double driveAngleDegrees = 0;

    ILT_Teleop mainScript;
    Motors motorHandler;
    StateMachine stateMachine;

    public void init(ILT_Teleop main, Motors motor, StateMachine SM) {
        mainScript = main;
        motorHandler = motor;
        stateMachine = SM;
    }
    void HandleInput(Gamepad gamepad1, Gamepad gamepad2) {
        strafeInput = gamepad1.left_stick_x;
        driveInput = -gamepad1.left_stick_y;
        turnInput = gamepad1.right_stick_x;
        if (gamepad1.left_bumper) {
            motorHandler.drive.setMaxSpeed(0.7);
            turnInput *= (0.5/0.7);
        } else {
            if (Math.abs(driveInput) > 0.7 && Math.abs(strafeInput) < 0.3) strafeInput = 0;
            motorHandler.drive.setMaxSpeed(1);
        }

        if (gamepad2.right_trigger > 0.4 && gamepad2.left_trigger > 0.4) {
            follower.setPose(mainScript.poseResetPose);
        }
        if (gamepad2.right_stick_button && gamepad2.left_bumper) motorHandler.turret.resetEncoder();

        if (gamepad1.left_stick_button) mainScript.turretTargetPos = 0;

        //Intake Section
        if (gamepad2.a || gamepad1.a) {
            stateMachine.intakeState = "intaking";
            stateMachine.launcherState = "rejecting";
        } else if (gamepad2.y || gamepad1.y) {
            stateMachine.intakeState = "rejecting";
            stateMachine.launcherState = "rejecting";
        } else if (gamepad2.b || gamepad1.b) {
            stateMachine.intakeState = "idle";
            if (Objects.equals(stateMachine.launcherState,"rejecting")) {
                stateMachine.launcherState = "idle";
            }
        } else if (gamepad1.right_stick_button) {
            stateMachine.intakeState = "firing";
        } else if (gamepad1.x || gamepad2.x) {
            stateMachine.intakeState = "idle";
            stateMachine.turretState = "idle";
            stateMachine.launcherState = "idle";
        }

        if (gamepad2.dpad_up) {
            stateMachine.launcherState = "testSpeed";
        } else if (gamepad2.dpad_down) {
            stateMachine.launcherState = "idle";
        }
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            stateMachine.launcherState = "beginLaunchSequence";
            stateMachine.turretState = "aiming";
            stateMachine.intakeState = "idle";
            mainScript.launchTimer.reset();
        }

        //Turret Section
        if (Math.abs(gamepad2.left_stick_x) > 0.1 || gamepad2.left_bumper) {
            Configurables.turretManualControl = true;
            if (motorHandler.turret.getCurrentPosition() > motorHandler.turretAngleToTicks(-135) && gamepad2.left_stick_x < 0) {
                motorHandler.turret.set(gamepad2.left_stick_x * .4);
            } else if (motorHandler.turret.getCurrentPosition() < motorHandler.turretAngleToTicks(135) && gamepad2.left_stick_x > 0) {
                motorHandler.turret.set(gamepad2.left_stick_x * .4);
            } else {
                motorHandler.turret.set(0);
            }
        } else Configurables.turretManualControl = false;

        if (gamepad2.dpad_left && driftAdjustToggle) { // in case turret drifts, player 2 clicks left or right for autoaim adjustment!!
            driftAdjustToggle = false;
            mainScript.turretDriftOffset -= 1;
        } else if (gamepad2.dpad_right && driftAdjustToggle) {
            driftAdjustToggle = false;
            mainScript.turretDriftOffset += 1;
        } else {
            driftAdjustToggle = true;
        }

        driveAngleDegrees = Math.toDegrees(mainScript.currentPose.getHeading());
    }
}
