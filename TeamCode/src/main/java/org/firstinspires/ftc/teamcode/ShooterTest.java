package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name = "ShootAS",group = "AS")
public class ShooterTest extends OpMode {
    public MotorEx shooter;
    //DcMotor shooter1 = shooter.motorEx;
    @Override
    public void init() {
        shooter = new MotorEx(hardwareMap,"shooter1", Motor.GoBILDA.BARE);
    }

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        if (gamepad1.a) {
            shooter.set(0.5);
        }
        if (gamepad1.b) {
            shooter.set(-0.5);
        }
        if (gamepad1.x) {
            shooter.set(0.85);
        }
        if (gamepad1.y) {
            shooter.set(-0.85);
        }
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {

    }
}
