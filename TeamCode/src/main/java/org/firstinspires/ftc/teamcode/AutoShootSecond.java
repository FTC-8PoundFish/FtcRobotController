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
import static org.firstinspires.ftc.config.SecondConfig.SECOND_JIGGLE_HALF_MS;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_JIGGLE_INCHES;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_JIGGLE_STOP_MS;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_FLOWER_DOWN_MS;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_WAIT_TAG;
import org.firstinspires.ftc.config.SecondConfig;
import static org.firstinspires.ftc.config.SecondConfig.secondLauncherMinVelocity;
import static org.firstinspires.ftc.config.SecondConfig.secondLauncherTargetVelocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * Launcher speeds come from LauncherConfig (tunable on FTC Dashboard).
 */
@Autonomous(name = "AutoShootSecond", group = "OpMode")
public class AutoShootSecond extends OpMode {

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

    // Start shooting as soon as the Limelight sees the tag we're waiting for. If it still
    // hasn't shown up when less than PARK_RESERVE_MS of the auto is left, skip shooting and
    // just park. SecondConfig.SECOND_WAIT_TAG (FTC Dashboard dropdown) decides what we wait for:
    //   BLUE -> wait for any blue tag (42-45)
    //   RED  -> wait for any red tag (30-33)
    //   ANY  -> wait for any tag ID that wasn't visible during init
    private static final int APRILTAG_PIPELINE = 1;
    // The autonomous period is 30s. Once less than PARK_RESERVE_MS is left we head straight
    // to park so we make it in time.
    private static final long AUTO_LENGTH_MS = 30_000;
    private static final long PARK_RESERVE_MS = 8_000;
    private static final int RED_TAG_MIN = 30, RED_TAG_MAX = 33;
    private static final int BLUE_TAG_MIN = 42, BLUE_TAG_MAX = 45;
    private Limelight3A limelight;
    private final Set<Integer> knownTagIds = new HashSet<>();
    private boolean watchingForNewTag = false;
    private String waitingFor = "any new tag";  // "red", "blue" or "any new tag"
    private int newTagId = -1;

    private static boolean isRed(int id) {
        return id >= RED_TAG_MIN && id <= RED_TAG_MAX;
    }
    private static boolean isBlue(int id) {
        return id >= BLUE_TAG_MIN && id <= BLUE_TAG_MAX;
    }

    private void chooseTagToWaitFor() {
        if (SECOND_WAIT_TAG == SecondConfig.WaitTag.BLUE) {
            waitingFor = "blue";
        } else if (SECOND_WAIT_TAG == SecondConfig.WaitTag.RED) {
            waitingFor = "red";
        } else {
            waitingFor = "any new tag";
        }
    }


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


    private long msLeft() {
        return AUTO_LENGTH_MS - (System.currentTimeMillis() - startTime);
    }

    private boolean lowOnTime() {
        return msLeft() < PARK_RESERVE_MS;
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
        Command shootToFlower = sequential(
                instant(() -> {
                    autoStage = "going to flower";
                    // Intake on, same as left_trigger in DriveTest.
                    intake.setPower(1);
                    leftIntakeServo.setPower(1);
                    rightIntakeServo.setPower(-1);
                }),
                follow(follower, toFlower()));
        Command flowerThenPark = sequential(
                shootToFlower,
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
                    autoStage = "parking";
                }),
                follow(follower, flowerToPark()));
        // Tag seen: shoot, then go to the flower if there's time (and SECOND_SHOOT is on),
        // otherwise park straight from shootPose.
        Command shootThenContinue = sequential(
                driveToShoot,
                instant(() -> {
                    shooting = SECOND_SHOOT;
                    if (shooting) {
                        autoStage = "launching tag " + newTagId + ":" + secondLauncherTargetVelocity();
                        launcher.setVelocity(secondLauncherTargetVelocity());
                        launching = true;  // loop() feeds the windmill while this is true
                    } else {
                        autoStage = "not shooting (SECOND_SHOOT off), waiting " + SHOOT_TIME + "ms";
                    }
                }),
                // Built when reached, so a dashboard change to SHOOT_TIME applies.
                lazy(() -> waitMs(SHOOT_TIME)),
                instant(() -> {
                    launching = false;
                    launcher.setVelocity(0);
                    windmillFeeder.stop();
                }),
                lazy(() -> {
                    if (!shooting) {
                        return parkFromShoot("no flower, SECOND_SHOOT off");
                    }
                    if (lowOnTime()) {
                        return parkFromShoot("only " + msLeft() + "ms left, skipping flower");
                    }
                    return flowerThenPark;
                }));
        // Tag never seen before time ran low: don't shoot, drive straight through to park.
        Command noTagPark = sequential(
                instant(() -> autoStage = "no " + waitingFor + " tag, parking without shooting"),
                driveToShoot,
                parkFromShoot("no tag seen"));
        return sequential(
                instant(() -> {
                    chooseTagToWaitFor();
                    watchingForNewTag = true;  // loop() sets newTagId when that tag shows up
                    autoStage = "waiting for " + waitingFor + " AprilTag";
                }),
                waitUntil(() -> newTagId >= 0 || lowOnTime()),
                instant(() -> watchingForNewTag = false),
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

    // Tags visible before we start watching count as "already there"; after that, we look
    // for a tag matching waitingFor.
    private void updateTags() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (!watchingForNewTag) {
                knownTagIds.add(id);
            } else if (newTagId < 0 && isTagWeAreWaitingFor(id)) {
                newTagId = id;
            }
        }
    }

    private boolean isTagWeAreWaitingFor(int id) {
        if (waitingFor.equals("red")) {
            return isRed(id);
        }
        if (waitingFor.equals("blue")) {
            return isBlue(id);
        }
        return !knownTagIds.contains(id);
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
        telemetry.addData("New tag", newTagId >= 0 ? String.valueOf(newTagId) : "none");

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

