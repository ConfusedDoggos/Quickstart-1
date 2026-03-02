/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.prototyping;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Sensor Testing + Servo Blocker", group="Linear OpMode")
@Configurable
public class AbsoluteEncoder extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //Analog Encoder Variables
    AnalogInput absEncoder;
    public static double zeroPositionOffset = -53;
    double outputVoltage=0;
    double outputAngle=0;
    double totalAngle = 0;
    double angleOffset = 0;
    double previousAngle = 0;
    private DistanceSensor distanceSensor1, distanceSensor2;
    private ColorSensor colorSensor;

    public ServoEx blockerServo;
    public static double closePosition = 250;
    public static double openPosition = 315;
    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        absEncoder = hardwareMap.get(AnalogInput.class,"AbsoluteEncoder");
        blockerServo = new ServoEx(hardwareMap,"blockerServo",0,300);
        blockerServo.setPwm(new PwmControl.PwmRange(500,2500));
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "firstDistanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "thirdDistanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) blockerServo.set(closePosition);
            if (gamepad1.b) blockerServo.set(openPosition);
            if (gamepad1.x) blockerServo.disable();
            outputVoltage = absEncoder.getVoltage();
            outputAngle = outputVoltage / 5 * 360;
            if (Math.abs(previousAngle - outputAngle) > 300) {
                if (outputAngle < 120) angleOffset += 360;
                if (outputAngle > 240) angleOffset -= 360;
            }
            totalAngle = outputAngle + angleOffset;
            telemetryM.addData("Distance 1",distanceSensor1.getDistance(DistanceUnit.INCH));
            telemetryM.addData("Color Alpha 2",colorSensor.alpha());
            telemetryM.addData("Distance 3",distanceSensor2.getDistance(DistanceUnit.INCH));
            telemetryM.addData("Output Voltage",outputVoltage);
            telemetryM.addData("Physical Angle",outputAngle);
            telemetryM.addData("Total Angle", totalAngle);
            telemetryM.addData("Turret Angle",(totalAngle + zeroPositionOffset)/2 );

            if (gamepad2.right_stick_button) angleOffset = 0;

            previousAngle = outputAngle;
            telemetryM.update(telemetry);
        }
    }
}