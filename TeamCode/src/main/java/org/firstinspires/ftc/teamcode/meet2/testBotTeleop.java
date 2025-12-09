package org.firstinspires.ftc.teamcode.meet2;
/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "Test Bot")
@Configurable
public class testBotTeleop extends LinearOpMode {



    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    private MotorEx fL, fR, bL, bR,launcher1,launcher2;
    private MotorGroup launcher;
    private ElapsedTime teleTimer;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private boolean driveMode = false;
    private double driveTrainSpeed = 1.0;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        LoopTimer timer = new LoopTimer();

        //set the default settings for motors and initializes them (wow)
        initMotors();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        teleTimer = new ElapsedTime();

        waitForStart();
        if (opModeIsActive()) {
            teleTimer.reset();
            while (opModeIsActive()) {
                hubs.forEach(LynxModule::clearBulkCache);
                timer.start();

                if (gamepad1.dpad_left) {
                    driveMode = true;
                } else if (gamepad1.dpad_right) {
                    driveMode = false;
                }

                if (gamepad1.left_bumper) {
                    driveTrainSpeed = 0.5;
                }
                else {
                    driveTrainSpeed = 1.0;
                }

                if (driveMode) {
                    drive.driveRobotCentric(
                            -gamepad1.left_stick_x * driveTrainSpeed,
                            gamepad1.left_stick_y * driveTrainSpeed,
                            -gamepad1.right_stick_x * driveTrainSpeed
                    );
                } else {
                    if (gamepad1.x) {
                        bL.set(1);
                        bR.set(0);
                        fL.set(0);
                        fR.set(0);
                    } else if (gamepad1.y) {
                        bR.set(1);
                        bL.set(0);
                        fL.set(0);
                        fR.set(0);
                    } else if (gamepad1.a) {
                        fL.set(1);
                        bL.set(0);
                        bR.set(0);
                        fR.set(0);
                    } else if (gamepad1.b) {
                        fR.set(1);
                        bL.set(0);
                        bR.set(0);
                        fL.set(0);
                    }
                    if (gamepad1.right_stick_button) {
                        bL.set(0);
                        bR.set(0);
                        fL.set(0);
                        fR.set(0);
                    }
                }
                launcherTeleOp();

                timer.end();
                //graphM.update();
                telemetryM.update(telemetry);
                // Share the CPU.

            }
        }

    }   // end method runOpMode()


    private void initMotors() {
        fL = new MotorEx(hardwareMap,"fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hardwareMap,"fR",Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hardwareMap,"bL",Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hardwareMap,"bR",Motor.GoBILDA.RPM_312);
        launcher1 = new MotorEx(hardwareMap,"shooterMotor1", Motor.GoBILDA.BARE);
        launcher2 = new MotorEx(hardwareMap,"shooterMotor2", Motor.GoBILDA.BARE);
        launcher2.setInverted(true);
        launcher = new MotorGroup(launcher1,launcher2);

        drive = new MecanumDrive(fL, fR, bL, bR);
    }

    private void launcherTeleOp() {
        if (Math.abs(gamepad2.left_stick_x) > .1) {
            launcher.set(gamepad2.left_stick_x);
        }
        else {
            launcher.stopMotor();
        }
        telemetryM.addData("Motor Velocity",launcher.getVelocity());
    }



}   // end class