package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name = "ShootAS",group = "AS")
@Disabled
public class ShooterTest extends OpMode {
    public MotorEx shooter, intake;
    private MotorEx fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    public static double kp = 0.05;
    public static double ki = 0.01;
    public static double kd = 0.31;
    public static double ks = 0.01;
    public static double kv = 0.31;
    public double speed;

    @Override
    public void init() {
        shooter = new MotorEx(hardwareMap,"shooterMotor", Motor.GoBILDA.BARE);
        intake = new MotorEx(hardwareMap,"intakeMotor", Motor.GoBILDA.RPM_435);
        fL = new MotorEx(hardwareMap,"fL",Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hardwareMap,"fR",Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hardwareMap,"bL",Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hardwareMap,"bR",Motor.GoBILDA.RPM_312);
        drive = new MecanumDrive(fL, fR, bL, bR);
        driverOp = new GamepadEx(gamepad1);
        /*shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(kp,ki,kd);
        shooter.setFeedforwardCoefficients();*/
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            speed = 0.5;
        } else {
            speed = 1;
        }
        drive.driveRobotCentric(
                -driverOp.getLeftX() * speed,
                -driverOp.getLeftY() * speed,
                -driverOp.getRightX() * speed
        );
        if (gamepad1.a) {
            shooter.set(0.5);
            telemetry.addData("A",gamepad1.a);
        } else if (gamepad1.b) {
            shooter.set(-0.5);
            telemetry.addData("B",gamepad1.b);
        } else {
            shooter.set(0);
        }
        if (gamepad1.x) {
            intake.set(1);
            telemetry.addData("X",gamepad1.x);
        } else if (gamepad1.y) {
            intake.set(-1);
            telemetry.addData("Y",gamepad1.y);
        } else {
            intake.set(0);
        }
        telemetry.addData("motor velocity",shooter.getVelocity());
        telemetry.update();
    }
}
