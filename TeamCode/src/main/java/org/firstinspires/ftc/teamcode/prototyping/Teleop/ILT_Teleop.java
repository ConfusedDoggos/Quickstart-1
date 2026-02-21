package org.firstinspires.ftc.teamcode.prototyping.Teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Meet_ILT.AprilTag;
import org.firstinspires.ftc.teamcode.meet3.Meet3Auto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.prototyping.Teleop.InputHandler;
import org.firstinspires.ftc.teamcode.prototyping.Teleop.Motors;
import org.firstinspires.ftc.teamcode.prototyping.Teleop.StateMachine;

import java.util.Objects;

@TeleOp(name="Chud Teleop", group="ILT")
@Disabled
public class ILT_Teleop extends LinearOpMode {

    // Declare OpMode members.
    public Pose currentPose;
    public Pose goalPose;
    public Pose poseResetPose;
    public int turretTargetPos;

    public double turretDriftOffset = 0;

    public static String team = Meet3Auto.team;

    public double angleError;
    public final ElapsedTime launchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private ElapsedTime runtime = new ElapsedTime();

    public InterpLUT rangeLUT = new InterpLUT();
    public InterpLUT velocityLUT = new InterpLUT();

    public Follower followerRef;



    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);

        followerRef = follower;

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


        //updateTeamDependents();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            follower.update();
            currentPose = follower.getPose();
            input.HandleInput(gamepad1, gamepad2);
            stateMachine.updateStates();
            telemetry.update();
        }


        }

}