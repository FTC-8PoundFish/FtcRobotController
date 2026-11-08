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
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//
//import java.util.HashSet;
//import java.util.Set;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.config.FlowerConfig.flowerHome;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_SHOOT;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_JIGGLE_HALF_MS;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_JIGGLE_INCHES;
import static org.firstinspires.ftc.config.SecondConfig.SECOND_JIGGLE_TIMES;
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
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(144, 84, 0);
    private final Pose flowerPose = p.of(133,100,0 );
    // move forward for 0 inches to shoot (shoot from startPose)
    private static final int forward = 0;
    private final Pose shootPose = p.of(144 - forward, 84, 0);
    private final Pose park = p.of(112, 130, 0);

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

    // Limelight is commented out for now: we just wait until TAG_WAIT_TIMEOUT_MS after
    // start before driving to shootPose.
    //
    // Start driving as soon as the Limelight sees the tag we're waiting for, or at
    // TAG_WAIT_TIMEOUT_MS after start if it never shows up. Tags seen during init and the
    // first 2s decide what we wait for:
    //   only blue seen -> wait for any red tag
    //   only red seen  -> wait for any blue tag
    //   otherwise      -> wait for any tag ID that wasn't visible before
//    private static final int APRILTAG_PIPELINE = 1;
    private static final long TAG_WAIT_TIMEOUT_MS = 10_000;
//    private static final int RED_TAG_MIN = 30, RED_TAG_MAX = 37;
//    private static final int BLUE_TAG_MIN = 38, BLUE_TAG_MAX = 45;
//    private Limelight3A limelight;
//    private final Set<Integer> knownTagIds = new HashSet<>();
//    private boolean watchingForNewTag = false;
//    private String waitingFor = "any new tag";  // "red", "blue" or "any new tag"
//    private int newTagId = -1;
//
//    private static boolean isRed(int id) {
//        return id >= RED_TAG_MIN && id <= RED_TAG_MAX;
//    }
//    private static boolean isBlue(int id) {
//        return id >= BLUE_TAG_MIN && id <= BLUE_TAG_MAX;
//    }
//
//    private void chooseTagToWaitFor() {
//        boolean sawRed = false, sawBlue = false;
//        for (int id : knownTagIds) {
//            sawRed |= isRed(id);
//            sawBlue |= isBlue(id);
//        }
//        if (sawBlue && !sawRed) {
//            waitingFor = "red";
//        } else if (sawRed && !sawBlue) {
//            waitingFor = "blue";
//        } else {
//            waitingFor = "any new tag";
//        }
//    }


    private Path startToShoot() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }
    private Path toFlower(){
        return line(shootPose,flowerPose).linear(shootPose,flowerPose);
    }
    private Path flowerToPark() {
        return line(flowerPose, park).linear(flowerPose, park);
    }


    private static boolean samePose(Pose a, Pose b) {
        return a.x() == b.x() && a.y() == b.y() && a.heading() == b.heading();
    }

    // Rock back and forth around wherever the robot is now (same as AutoShootGarden's
    // jiggle). Built when it starts, so it uses the robot's actual pose and the current
    // dashboard values.
    private Command jiggle() {
        Pose here = follower.pose();
        double heading = here.heading();
        Pose anchor = p.of(here.x(), here.y(), Math.toDegrees(heading));
        Pose back = p.of(
                here.x() - SECOND_JIGGLE_INCHES * Math.cos(heading),
                here.y() - SECOND_JIGGLE_INCHES * Math.sin(heading),
                Math.toDegrees(heading));
        return repeat(
                sequential(
                        race(follow(follower, line(anchor, back).linear(anchor, back)),
                                waitMs(SECOND_JIGGLE_HALF_MS)),
                        race(follow(follower, line(back, anchor).linear(back, anchor)),
                                waitMs(SECOND_JIGGLE_HALF_MS))),
                SECOND_JIGGLE_TIMES);
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
        Command driveToPark = follow(follower, flowerToPark());
        return sequential(
                instant(() -> autoStage = "waiting 2s"),
//                instant(() -> {
//                    chooseTagToWaitFor();
//                    watchingForNewTag = true;  // loop() sets newTagId when that tag shows up
//                    autoStage = "waiting for " + waitingFor + " AprilTag";
//                }),
//                waitUntil(() -> newTagId >= 0
//                        || System.currentTimeMillis() - startTime > TAG_WAIT_TIMEOUT_MS),
//                instant(() -> {
//                    watchingForNewTag = false;
//                    autoStage = newTagId >= 0
//                            ? "tag " + newTagId + " seen, driving to shootPose"
//                            : "no " + waitingFor + " tag in 10s, driving to shootPose";
//                }),
                instant(() -> autoStage = "waiting until 10s"),
                waitUntil(() -> System.currentTimeMillis() - startTime > TAG_WAIT_TIMEOUT_MS),
                instant(() -> autoStage = "driving to shootPose"),
                driveToShoot,
                instant(() -> {
//                    String tag = (newTagId >= 0) ? " detect_tag " : " timeout";
//                    autoStage = "launching" + tag;
                    if (SECOND_SHOOT) {
                        autoStage = "launching:" + secondLauncherTargetVelocity();
                        launcher.setVelocity(secondLauncherTargetVelocity());
                        launching = true;  // loop() feeds the windmill while this is true
                    } else {
                        autoStage = "not shooting (SECOND_SHOOT off), waiting 10s";
                    }
                }),
                waitMs(7_000),
                instant(() -> {
                    launching = false;
                    launcher.setVelocity(0);
                    windmillFeeder.stop();
                }),
                shootToFlower,
                instant(() -> autoStage = "jiggling at flower"),
                lazy(this::jiggle),
                instant(() -> autoStage = "parking"),
                driveToPark,
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
//        limelight.stop();
    }

//    // Tags visible before we start watching count as "already there"; after that, we look
//    // for a tag matching waitingFor.
//    private void updateTags() {
//        LLResult result = limelight.getLatestResult();
//        if (result == null || !result.isValid()) {
//            return;
//        }
//        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
//            int id = tag.getFiducialId();
//            if (!watchingForNewTag) {
//                knownTagIds.add(id);
//            } else if (newTagId < 0 && isTagWeAreWaitingFor(id)) {
//                newTagId = id;
//            }
//        }
//    }
//
//    private boolean isTagWeAreWaitingFor(int id) {
//        if (waitingFor.equals("red")) {
//            return isRed(id);
//        }
//        if (waitingFor.equals("blue")) {
//            return isBlue(id);
//        }
//        return !knownTagIds.contains(id);
//    }

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
        rightFlowerIntakeServo.setPosition(flowerHome());
        leftflowerIntakeServo.setPosition(flowerHome());
        launcher =hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100);
//        limelight.pipelineSwitch(APRILTAG_PIPELINE);
//        limelight.start();
        telemetry.addData(
                "Speed",
                "target (%d), min (%d)",
                secondLauncherTargetVelocity(),
                secondLauncherMinVelocity());
        telemetry.update();
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

//    @Override
//    public void init_loop() {
//        updateTags();
//        telemetry.addData("Tags already visible", knownTagIds);
//        telemetry.update();
//    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {

        follower.update();
//        updateTags();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Auto stage", autoStage);
        telemetry.addData("Time (s)", "%.1f", (System.currentTimeMillis() - startTime) / 1000.0);
//        telemetry.addData("Tags already visible", knownTagIds);
//        telemetry.addData("New tag", newTagId >= 0 ? String.valueOf(newTagId) : "none");

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

