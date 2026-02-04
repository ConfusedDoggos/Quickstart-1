package org.firstinspires.ftc.teamcode.prototyping;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Meet_ILT.AprilTag;

@TeleOp(name="ILT Teleop", group="ILT")
public class ILT_Teleop extends LinearOpMode {

    // Declare OpMode members.
    public Pose currentPose;
    public Pose goalPose;
    public Pose poseResetPose;
    public int turretTargetPos;

    public double turretDriftOffset = 0;



    public double angleError;
    public final ElapsedTime launchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private ElapsedTime runtime = new ElapsedTime();

    public InterpLUT rangeLUT = new InterpLUT();
    public InterpLUT velocityLUT = new InterpLUT();


    @Override
    public void runOpMode() {
        AprilTag aprilTag = new AprilTag();
        Motors motors = new Motors();
        InputHandler input = new InputHandler();
        StateMachine stateMachine = new StateMachine();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        motors.init(hardwareMap, this);
        stateMachine.init(this,motors);
        input.init(this,motors,stateMachine);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            input.HandleInput(gamepad1, gamepad2);
            stateMachine.updateStates();
            telemetry.update();
        }
    }
}