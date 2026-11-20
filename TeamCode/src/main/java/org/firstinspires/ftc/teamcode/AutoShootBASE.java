package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.HashSet;
import java.util.Set;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.config.FlowerConfig.flower90Deg;
import static org.firstinspires.ftc.config.FlowerConfig.flowerHome;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_SHOOT;
import static org.firstinspires.ftc.config.SecondConfig.SHOOT_TIME;
import static org.firstinspires.ftc.config.SecondConfig.EARLY_TAG_MS;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_JIGGLE_HALF_MS;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_JIGGLE_INCHES;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_JIGGLE_STOP_MS;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_FLOWER_DOWN_MS;
import static org.firstinspires.ftc.config.SecondConfig.secondLauncherMinVelocity;
import static org.firstinspires.ftc.config.SecondConfig.secondLauncherTargetVelocity;

/*
 * Shared code for AutoShootRED and AutoShootBLUE. The only difference between them is which
 * AprilTags they wait for (allianceName() / isOurTag()).
 * Tunables are in SecondConfig (tunable on FTC Dashboard).
 */
public abstract class AutoShootBASE extends OpMode {

    private long startTime;

    private boolean launching = false;
    // SECOND_SHOOT as read at shootPose; also decides whether we go to the flower after.
    private boolean shooting = false;
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(144, 84, 0);
    private final Pose flowerPose = p.of(127,100,0 );
    // move forward for 0 inches to shoot (shoot from startPose)
    private static final int forward = 0;
    private final Pose shootPose = p.of(144 - forward, 84, 0);
    private final Pose park = p.of(127, 130, 0);
    private final Pose twiceTheFun = p.of(135,110, 33);

    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    // Flower positions come from FlowerConfig (tunable on FTC Dashboard), same as DriveTest.
    private Servo rightFlowerIntakeServo = null;
    private Servo leftflowerIntakeServo = null;
    private DcMotor neoWindmill = null;
    double intakePower;

    private GoBildaPinpointDriver odo;

    private WindmillFeeder windmillFeeder;

    public static int Launcher_Velocity = 0;

    // Set by each step of autoRoutine() and shown every loop.
    private String autoStage = "waiting for start";

    // Start shooting as soon as the Limelight sees one of our alliance's tags (isOurTag()).
    // If it still hasn't shown up when less than PARK_RESERVE_MS of the auto is left, skip
    // shooting and just park.
    private static final int APRILTAG_PIPELINE = 1;
    // The autonomous period is 30s. Once less than PARK_RESERVE_MS is left we head straight
    // to park so we make it in time.
    private static final long AUTO_LENGTH_MS = 30_000;
    private static final long PARK_RESERVE_MS = 8_000;
    // When the tag was seen (ms after start), compared against SecondConfig.EARLY_TAG_MS.
    private long tagSeenAtMs = -1;
    protected static final int RED_TAG_MIN = 30, RED_TAG_MAX = 33;
    protected static final int BLUE_TAG_MIN = 42, BLUE_TAG_MAX = 45;
    private Limelight3A limelight;
    private final Set<Integer> knownTagIds = new HashSet<>();
    private boolean watchingForNewTag = false;
    private int newTagId = -1;

    // "red" or "blue"; shown in telemetry.
    protected abstract String allianceName();

    // Whether this AprilTag ID is one of the tags we wait for before shooting.
    protected abstract boolean isOurTag(int id);


    private Path startToShoot() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }
    private Path toFlower(){
        return line(shootPose,flowerPose).linear(shootPose,flowerPose);
    }
    private Path flowerToPark() {
        return line(flowerPose, park).linear(flowerPose, park);
    }
    private Path shootToPark() {
        return line(shootPose, park).linear(shootPose, park);
    }
    private Path secondShot() {
        return line(flowerPose, twiceTheFun).linear(flowerPose, twiceTheFun);
    }

    private Path twiceThePark(){
        return line(twiceTheFun,park).linear(twiceTheFun,park);
    }

    private long msLeft() {
        return AUTO_LENGTH_MS - (System.currentTimeMillis() - startTime);
    }

    private boolean lowOnTime() {
        return msLeft() < PARK_RESERVE_MS;
    }

    // Shoot for SHOOT_TIME at the SecondConfig launcher speed, then stop the launcher.
    // Built fresh each time.
    private Command shoot(String label) {
        return sequential(
                instant(() -> {
                    autoStage = label + ":" + secondLauncherTargetVelocity();
                    launcher.setVelocity(secondLauncherTargetVelocity());
                    launching = true;  // loop() feeds the windmill while this is true
                }),
                // Built when reached, so a dashboard change to SHOOT_TIME applies.
                lazy(() -> waitMs(SHOOT_TIME)),
                instant(() -> {
                    launching = false;
                    launcher.setVelocity(0);
                    windmillFeeder.stop();
                }));
    }

    // shootPose -> flower, lower it, jiggle, raise it. Ends at flowerPose. Built fresh each time.
    private Command flowerTrip() {
        return sequential(
                instant(() -> {
                    autoStage = "going to flower";
                    // Intake on, same as left_trigger in DriveTest.
                    intake.setPower(1);
                    leftIntakeServo.setPower(1);
                    rightIntakeServo.setPower(-1);
                }),
                follow(follower, toFlower()),
                instant(() -> {
                    // Flower down, same as dpad_down in DriveTest.
                    double flowerDown = Math.max(0.0, Math.min(1.0, flowerHome() + flower90Deg()));
                    rightFlowerIntakeServo.setPosition(flowerDown);
                    leftflowerIntakeServo.setPosition(flowerDown);
                    autoStage = "lowering flower";
                }),
                // Servos don't report position, so give them time to get all the way down.
                lazy(() -> waitMs(SECOND_FLOWER_DOWN_MS)),
                instant(() -> autoStage = "jiggling at flower"),
                lazy(this::jiggle),
                instant(() -> {
                    // Flower back up to home, same as dpad_up in DriveTest.
                    rightFlowerIntakeServo.setPosition(flowerHome());
                    leftflowerIntakeServo.setPosition(flowerHome());
                }));
    }

    // Drive from shootPose straight to park, no stops. Built fresh each time.
    private Command parkFromShoot(String why) {
        return sequential(
                instant(() -> autoStage = "parking (" + why + ")"),
                follow(follower, shootToPark()));
    }

    private static boolean samePose(Pose a, Pose b) {
        return a.x() == b.x() && a.y() == b.y() && a.heading() == b.heading();
    }

    // One rock from wherever the robot is now: drive forward SECOND_JIGGLE_INCHES, stop for
    // SECOND_JIGGLE_STOP_MS, drive back. Built when it starts, so it uses the robot's actual
    // pose and the current dashboard values.
    private Command jiggle() {
        Pose here = follower.pose();
        double heading = here.heading();
        Pose anchor = p.of(here.x(), here.y(), Math.toDegrees(heading));
        Pose ahead = p.of(
                here.x() + SECOND_JIGGLE_INCHES * Math.cos(heading),
                here.y() + SECOND_JIGGLE_INCHES * Math.sin(heading),
                Math.toDegrees(heading));
        return sequential(
                race(follow(follower, line(anchor, ahead).linear(anchor, ahead)),
                        waitMs(SECOND_JIGGLE_HALF_MS)),
                waitMs(SECOND_JIGGLE_STOP_MS),
                race(follow(follower, line(ahead, anchor).linear(ahead, anchor)),
                        waitMs(SECOND_JIGGLE_HALF_MS)));
    }

    // The whole auto. Each step starts as soon as the previous one finishes.
    private Command autoRoutine() {
        // With forward = 0 we already start at shootPose, so don't build or follow a
        // zero-length path.
        Command driveToShoot = samePose(startPose, shootPose)
                ? instant(() -> autoStage = "already at shootPose")
                : follow(follower, startToShoot());
        Command flowerThenPark = sequential(
                flowerTrip(),
                instant(() -> autoStage = "parking"),
                follow(follower, flowerToPark()));
        // AutoShootTwice's route: flower, to twiceTheFun, shoot again, park.
        Command flowerThenShootAgain = sequential(
                flowerTrip(),
                instant(() -> autoStage = "going to twiceTheFun for second shot"),
                follow(follower, secondShot()),
                shoot("second shot"),
                // secondShot() ends at twiceTheFun, not shootPose, so park from there.
                instant(() -> autoStage = "parking (after second shot)"),
                follow(follower, twiceThePark()));
        // Tag seen: shoot, then go to the flower if there's time (and SECOND_SHOOT is on),
        // otherwise park straight from shootPose.
        Command shootThenContinue = sequential(
                driveToShoot,
                lazy(() -> {
                    shooting = SECOND_SHOOT;
                    if (shooting) {
                        return shoot("launching tag " + newTagId);
                    }
                    autoStage = "not shooting (SECOND_SHOOT off), waiting " + SHOOT_TIME + "ms";
                    return waitMs(SHOOT_TIME);
                }),
                lazy(() -> {
                    if (!shooting) {
                        return parkFromShoot("no flower, SECOND_SHOOT off");
                    }
                    if (lowOnTime()) {
                        return parkFromShoot("only " + msLeft() + "ms left, skipping flower");
                    }
                    if (tagSeenAtMs < EARLY_TAG_MS) {
                        return flowerThenShootAgain;
                    }
                    return flowerThenPark;
                }));
        // Tag never seen before time ran low: don't shoot, drive straight through to park.
        Command noTagPark = sequential(
                instant(() -> autoStage = "no " + allianceName() + " tag, parking without shooting"),
                driveToShoot,
                parkFromShoot("no tag seen"));
        return sequential(
                instant(() -> {
                    watchingForNewTag = true;  // loop() sets newTagId when that tag shows up
                    autoStage = "waiting for " + allianceName() + " AprilTag";
                }),
                waitUntil(() -> newTagId >= 0 || lowOnTime()),
                instant(() -> {
                    watchingForNewTag = false;
                    if (newTagId >= 0) {
                        tagSeenAtMs = System.currentTimeMillis() - startTime;
                    }
                }),
                lazy(() -> newTagId >= 0 ? shootThenContinue : noTagPark),
                instant(() -> {
                    intake.setPower(0);
                    leftIntakeServo.setPower(0);
                    rightIntakeServo.setPower(0);
                    autoStage = "parked";
                })
        );
    }


    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        limelight.stop();
    }

    // Before we start watching, just record which tags are visible (shown in telemetry);
    // after that, look for one of our alliance's tags.
    private void updateTags() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (!watchingForNewTag) {
                knownTagIds.add(id);
            } else if (newTagId < 0 && isOurTag(id)) {
                newTagId = id;
            }
        }
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
        neoWindmill = hardwareMap.get(DcMotor.class, "NEO");
        neoWindmill.setDirection(DcMotorSimple.Direction.REVERSE);
        neoWindmill.setPower(0);
        windmillFeeder = new WindmillFeeder(neoWindmill);
        // Intake and flower setup, copied from DriveTest.
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);
        leftIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        rightFlowerIntakeServo = hardwareMap.get(Servo.class, "flowery");
        leftflowerIntakeServo = hardwareMap.get(Servo.class, "flowey");
        rightFlowerIntakeServo.setDirection(Servo.Direction.REVERSE);
        leftflowerIntakeServo.setDirection(Servo.Direction.FORWARD);
        telemetry.addData(
                "Speed",
                "target (%d), min (%d) flower_left (%f), flower_right (%f)",
                secondLauncherTargetVelocity(),
                secondLauncherMinVelocity(),
                leftflowerIntakeServo.getPosition(),
                rightFlowerIntakeServo.getPosition());
        telemetry.update();

        rightFlowerIntakeServo.setPosition(flowerHome());
        leftflowerIntakeServo.setPosition(flowerHome());
        launcher =hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
//        if (gamepad1.right_bumper) {
//            long start = System.currentTimeMillis();
//            while (System.currentTimeMillis() - start < 500) {
//            }
//            launcherTargetVelocity += 50;
//            launcherMinVelocity += 50;
//            telemetry.addData(
//                    "Speed",
//                    "target (%d), min (%d)",
//                    launcherTargetVelocity,
//                    launcherMinVelocity);
//            telemetry.update();
//        }
//        if (gamepad1.right_trigger_pressed) {
//            long start = System.currentTimeMillis();
//            while (System.currentTimeMillis() - start < 500) {
//            }
//            launcherTargetVelocity += 50;
//            launcherMinVelocity += 50;
//            telemetry.addData(
//                    "Speed",
//                    "target (%d), min (%d)",
//                    launcherTargetVelocity,
//                    launcherMinVelocity);
//            telemetry.update();
//        }
    }

    @Override
    public void init_loop() {
        updateTags();
        telemetry.addData("Tags already visible", knownTagIds);
        telemetry.update();
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        launcher.setVelocity(secondLauncherTargetVelocity());
        intake.setPower(1);
        rightIntakeServo.setPower(1);
        leftIntakeServo.setPower(1);

        follower.update();
        updateTags();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Auto stage", autoStage);
        telemetry.addData("Time (s)", "%.1f", (System.currentTimeMillis() - startTime) / 1000.0);
        telemetry.addData("Tags already visible", knownTagIds);
        telemetry.addData("New tag", newTagId >= 0
                ? newTagId + " at " + tagSeenAtMs + "ms" + (tagSeenAtMs < EARLY_TAG_MS ? " (early: shoot twice)" : "")
                : "none");

        if (launching) {
            if (launcher.getVelocity() > secondLauncherMinVelocity()) {
                telemetry.addData("launch start", "launch_speed_reached");
                windmillFeeder.feed();
            } else {
                telemetry.addData("launch start", "launch_speed_not_reached");
                windmillFeeder.stop();
            }
        }

        telemetry.update();
    }
}

