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
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//
//import java.util.HashSet;
//import java.util.Set;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import static org.firstinspires.ftc.teamcode.LauncherConfig.launcherMinVelocity;
import static org.firstinspires.ftc.teamcode.LauncherConfig.launcherTargetVelocity;

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
    // move forward for 0 inches to shoot (shoot from startPose)
    private static final int forward = 0;
    private final Pose shootPose = p.of(144 - forward, 84, 0);
    private final Pose park = p.of(132, 130, 0);
    // If we shoot from closer to the wall than this x, first drive forward to this x
    // before parking so we don't bump into the flower.
    private static final double CLEAR_X = 138;
    private final Pose clearPose = p.of(CLEAR_X, 84, 0);

    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
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
    private Path Park() {
        return line(shootPose, park).linear(shootPose, park);
    }
    private Path shootToClear() {
        return line(shootPose, clearPose).linear(shootPose, clearPose);
    }
    private Path clearToPark() {
        return line(clearPose, park).linear(clearPose, park);
    }


    private static boolean samePose(Pose a, Pose b) {
        return a.x() == b.x() && a.y() == b.y() && a.heading() == b.heading();
    }

    // The whole auto. Each step starts as soon as the previous one finishes.
    private Command autoRoutine() {
        // With forward = 0 we already start at shootPose, so don't build or follow a
        // zero-length path.
        Command driveToShoot = samePose(startPose, shootPose)
                ? instant(() -> autoStage = "already at shootPose")
                : follow(follower, startToShoot());
        Command driveToPark = shootPose.x() > CLEAR_X
                ? sequential(
                        follow(follower, shootToClear()),
                        instant(() -> autoStage = "parking (cleared flower)"),
                        follow(follower, clearToPark()))
                : follow(follower, Park());
        return sequential(
                instant(() -> autoStage = "waiting 2s"),
                waitMs(2000),
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
                    autoStage = "launching";
                    launcher.setVelocity(launcherTargetVelocity);
                    launching = true;  // loop() feeds the windmill while this is true
                }),
                waitMs(10_000),
                instant(() -> {
                    launching = false;
                    launcher.setVelocity(0);
                    windmillFeeder.stop();
                    autoStage = "parking";
                }),
                driveToPark,
                instant(() -> autoStage = "parked")
        );
    }

//    private Path park() {
//        return curve(startPose, controlPose, park).linear(startPose, park);
//    }
// in your autonomous


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
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setPower(0);
        windmillFeeder = new WindmillFeeder(windmillServo);
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
                launcherTargetVelocity,
                launcherMinVelocity());
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
            if (launcher.getVelocity() > launcherMinVelocity()) {
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

