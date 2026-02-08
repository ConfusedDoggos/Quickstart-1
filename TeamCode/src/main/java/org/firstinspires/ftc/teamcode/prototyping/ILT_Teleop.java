package org.firstinspires.ftc.teamcode.prototyping;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Meet_ILT.AprilTag;
import org.firstinspires.ftc.teamcode.meet3.Meet3Auto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;

@TeleOp(name="ILT Teleop", group="ILT")
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

        while (opModeInInit()) {
            telemetry.addLine("Driver: Press left stick for blue team and right stick for red team.");
            telemetry.addData("Team Selected:",team);
            telemetry.addData("Use Base start position(testing)",useRealStart);
            telemetry.update();
            if (gamepad1.left_stick_button) {
                team="blue";
            } else if (gamepad1.right_stick_button) {
                team="red";
            }
            if (gamepad1.a) {
                useRealStart = true;
            }
        }
        updateTeamDependents();

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
    public void updateTeamDependents() {
        if (Objects.equals(team,"blue")){
            goalID = 20;
            startPose = new Pose(18.5,118.5,Math.toRadians(143+180));
            goalPose = new Pose(0,144,Math.toRadians(135));
            poseResetPose = new Pose(114,7,Math.toRadians(90)); //need to find good one
            aprilTagPose = new Pose(15,130,0);
        } else if (Objects.equals(team,"red")) {
            goalID = 24;
            startPose = new Pose(x(18.5),118.5,a(143+180));
            goalPose = new Pose(144,144,Math.toRadians(45));
            poseResetPose = new Pose(30,7,Math.toRadians(90)); //need to find good one
            aprilTagPose = new Pose(129,130,0);
        }
}