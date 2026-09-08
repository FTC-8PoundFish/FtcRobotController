package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.util.*;


import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "8lb Fish Challenge 6")
public class Challenge_6 extends OpMode {

    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private DcMotorEx intake;

    private Timer pathTimer, OpModeTimer;
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight3A;

    private Follower follower;

    /**
     * The path states, in the numerical order they run. The declaration order IS the run order --
     * each constant's ordinal() is its step number (First = 0 ... Fourth = 3). To add a leg of the
     * route, drop a new constant in at the position you want it to run; nothing else needs renumbering.
     */
    private enum PathState {
        First,

        Seccond,

        Third,

        Fourth

    }

    PathState pathState;

    /** True once we've handed the current state's path to the follower, so we only send it once. */
    private boolean pathStarted;

    /** How many full laps of First..Fourth we've completed. */
    private int lapsCompleted;

    /** Set to 0 (or less) to loop forever; otherwise the OpMode stops after this many laps. */
    private static final int MAX_LAPS = 0;

    /** Ignore isBusy() for this long after starting a path, so we can't "finish" it instantly. */
    private static final double MIN_PATH_SECONDS = 0.1;

    private final Pose leftBackPos = new Pose(0,0, Math.toRadians(0));
    private final Pose rightBackPos = new Pose(20,0,Math.toRadians(90));
    private final Pose rightFrontPos = new Pose(20,20,Math.toRadians(180));
    private final Pose leftFrontPos = new Pose(0,20,Math.toRadians(270));
    private PathChain first, seccond, third, fourth ;

    public void buildPaths(){
        first =
                follower.pathBuilder()
                        .addPath(new BezierLine(leftBackPos, rightBackPos))
                        .setLinearHeadingInterpolation(leftBackPos.getHeading(), rightBackPos.getHeading())
                        .build();
        seccond =
                follower.pathBuilder()
                        .addPath(new BezierLine(rightBackPos, rightFrontPos))
                        .setLinearHeadingInterpolation(rightBackPos.getHeading(), rightFrontPos.getHeading())
                        .build();
        third =
                follower.pathBuilder()
                        .addPath(new BezierLine(rightFrontPos, leftFrontPos))
                        .setLinearHeadingInterpolation(rightFrontPos.getHeading(), leftFrontPos.getHeading())
                        .build();
        fourth =
                follower.pathBuilder()
                        .addPath(new BezierLine(leftFrontPos, leftBackPos))
                        .setLinearHeadingInterpolation(leftFrontPos.getHeading(), leftBackPos.getHeading())
                        .build();
    }

    /** The PathChain that belongs to each state. */
    public PathChain pathFor(PathState state){
        switch (state){
            case First:   return first;
            case Seccond: return seccond;
            case Third:   return third;
            case Fourth:  return fourth;
            default:      return null;
        }
    }

    /**
     * The state one step further along, wrapping from the last one back to the first.
     * values() is in declaration order, so ordinal() + 1 mod length is "the next step, looped".
     */
    public PathState nextState(PathState state){
        PathState[] order = PathState.values();
        return order[(state.ordinal() + 1) % order.length];
    }

    /** True when the state we just advanced into is the start of a fresh lap. */
    public boolean wrappedAround(PathState state){
        return state.ordinal() == 0;
    }

    /** Enter a state: remember it, restart the path timer, and arm it to be started. */
    public void setPathState(PathState newState){
        pathState = newState;
        pathStarted = false;
        pathTimer.resetTimer();
    }

    /** True once the follower has actually run the current path to completion. */
    public boolean currentPathFinished(){
        return pathStarted
                && pathTimer.getElapsedTimeSeconds() > MIN_PATH_SECONDS
                && !follower.isBusy();
    }

    /**
     * One tick of the state machine: start the current state's path if it hasn't been started,
     * then advance to the next state (looping back to First) once that path reports finished.
     */
    public void statePathUpdate(){
        PathChain path = pathFor(pathState);
        if (path == null) {
            telemetry.addLine("No path for state " + pathState);
            return;
        }

        if (!pathStarted) {
            follower.followPath(path, true);
            pathStarted = true;
            telemetry.addLine("Started " + pathState);
            return;
        }

        if (currentPathFinished()) {
            telemetry.addLine("Yippeee! " + pathState + " Done");

            PathState next = nextState(pathState);
            if (wrappedAround(next)) {
                lapsCompleted++;
                if (MAX_LAPS > 0 && lapsCompleted >= MAX_LAPS) {
                    telemetry.addLine("Finished " + lapsCompleted + " lap(s) -- stopping");
                    requestOpModeStop();
                    return;
                }
            }
//            setPathState(next);
        }
    }


    @Override
    public void init() {

        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(9); //current color tracking pipeline

//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
//        odo.setOffsets(
//                -84.0,
//                -168.0,
//                DistanceUnit.MM); // these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(
//                GoBildaPinpointDriver.EncoderDirection.REVERSED,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();
        telemetry.addData("Status", "Initialized");
//        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
//        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
//        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();

        pathTimer = new Timer();
        OpModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(leftBackPos);

        pathState = PathState.First;
        pathStarted = false;
        lapsCompleted = 0;
    }

    public void start() {

        limelight3A.start();

        OpModeTimer.resetTimer();
        // Re-enter the first state so its path gets started on the first loop() tick.
        setPathState(PathState.First);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        odo.update();
        Pose2D pos = odo.getPosition();

        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry.addData("Target X offset", llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target Area percent", llResult.getTa());
        }

        telemetry.addData("Path State", pathState + " (step " + (pathState.ordinal() + 1)
                + " of " + PathState.values().length + ")");
        telemetry.addData("Next State", nextState(pathState));
        telemetry.addData("Laps Completed", lapsCompleted);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode time", OpModeTimer.getElapsedTimeSeconds());
    }


}
