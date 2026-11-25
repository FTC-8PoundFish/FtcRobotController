package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.config.FlowerConfig.flowerHome;
import static org.firstinspires.ftc.config.LauncherConfig.autoLauncherMinVelocity;
import static org.firstinspires.ftc.config.LauncherConfig.autoLauncherTargetVelocity;
import static org.firstinspires.ftc.config.LauncherConfig.forward;
import static org.firstinspires.ftc.config.GardenConfig.GARDEN_DRIVE_MAX_MS;
import static org.firstinspires.ftc.config.GardenConfig.GARDEN_JIGGLE_HALF_MS;
import static org.firstinspires.ftc.config.GardenConfig.GARDEN_JIGGLE_INCHES;
import static org.firstinspires.ftc.config.GardenConfig.GARDEN_JIGGLE_TIMES;
import static org.firstinspires.ftc.config.GardenConfig.SHOOT_HEADING;
import static org.firstinspires.ftc.config.GardenConfig.GARDEN_PARK_X;
import static org.firstinspires.ftc.config.GardenConfig.GARDEN_PARK_Y;

/*
 * Three-step auto. Each step starts as soon as the previous one finishes (see autoRoutine()):
 *   1. Drive to shootPose (if it isn't startPose), wait until the Limelight sees one of
 *      AprilTags 34-41, then shoot for SHOOT_MS.
 *   2. toGardenAproach, toGarden, and jiggle, same as GardenAutoBase.
 *   3. thirdShot back to shootPose and shoot for SHOOT_MS.
 *   4. parking.
 * Time limits (time left in the 30s auto):
 *   - Under SKIP_GARDEN_MS (10s) after the first shot: skip 2-3 and park.
 *   - Under PARK_NOW_MS (5s) at any point, tag or no tag: stop everything and park.
 *   Shots end early if needed so they never run into the last PARK_NOW_MS.
 * Tunables (SHOOT_HEADING, GARDEN_*) are in GardenConfig, tunable on FTC Dashboard.
 */
@Autonomous(name = "ThreeTips", group = "OpMode")
public class ThreeTips extends OpMode {
    protected long startTime;
    private int shootHeadingUsed = SHOOT_HEADING;

    // How long each shot feeds balls into the launcher (cut short if needed so the robot can
    // still start parking with PARK_NOW_MS left).
    private static final long SHOOT_MS = 5_000;
    // Time limits, counted against the 30s autonomous period:
    //   less than SKIP_GARDEN_MS left after the first shot -> skip the garden, park.
    //   less than PARK_NOW_MS left at any point -> drop everything and park.
    private static final long AUTO_LENGTH_MS = 30_000;
    private static final long SKIP_GARDEN_MS = 10_000;
    private static final long PARK_NOW_MS = 5_000;
    // Set once the robot starts parking, so the PARK_NOW_MS check fires only once.
    private boolean parking = false;
    // Start shooting once any of these AprilTags is seen.
    private static final int WAIT_TAG_MIN = 34, WAIT_TAG_MAX = 41;
    private static final int APRILTAG_PIPELINE = 1;  // same pipeline as AutoShootBASE

    // True while a shot is running; loop() feeds the windmill while it's set.
    private boolean launching = false;
    // The tag that started the first shot, or -1 before one is seen.
    private int seenTagId = -1;

    protected Follower follower;
    protected final PoseFactory p = PoseFactory.degrees();
    private final Pose shootPose = p.of(144-forward, 48, shootHeadingUsed);
    private final Pose flower = p.of(144,54,90);
    private final Pose startPose = p.of(144, 48, 0);

    protected final Pose gardenPose = p.of(144,0,0);
    private final Pose gardenAproach = p.of(130,0,0);
    protected final Pose park = p.of(GARDEN_PARK_X, GARDEN_PARK_Y, 0);

    // shootPose is set when INIT is pressed, so change SHOOT_HEADING or forward on the
    // dashboard before INIT.

    protected DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    // Flower positions come from FlowerConfig (tunable on FTC Dashboard), same as DriveTest.
    private Servo rightFlowerIntakeServo = null;
    private Servo leftflowerIntakeServo = null;
    protected DcMotor neoWindmill = null;
    protected double intakePower;
    private Limelight3A limelight;

    // Which step of the auto we're in; shown every loop.
    protected String phase = "waiting for start";

    private GoBildaPinpointDriver odo;

    protected WindmillFeeder windmillFeeder;

    private Path toFlower(){
        return line(shootPose,flower).linear(shootPose, flower);
    }
    private Path toGardenAproach(){
        return line(shootPose,gardenAproach).linear(shootPose, gardenAproach);
    }
    private Path toGarden(){
        return line(gardenAproach,gardenPose).linear(gardenAproach,gardenPose);
    }
    private Path startToShoot() {

        return line(startPose, shootPose).linear(startPose, shootPose);
    }

    private Path parking(){
        return line(shootPose,park).linear(shootPose,park);
    }
    private Path secondShot(){
        return line(flower,shootPose).linear(flower,shootPose);
    }
    private Path thirdShot(){
        return line(gardenPose,shootPose).linear(gardenPose,shootPose);
    }

    private static boolean samePose(Pose a, Pose b) {
        return a.x() == b.x() && a.y() == b.y() && a.heading() == b.heading();
    }

    // Drive into the garden, then rock back and forth around wherever the robot stopped.
    // Same as GardenAutoBase.
    private Command gardenAndJiggle() {
        return sequential(
                race(follow(follower, toGarden()), waitMs(GARDEN_DRIVE_MAX_MS)),
                instant(() -> phase = "jiggling in garden"),
                lazy(this::jiggle),
                instant(() -> phase = "in garden"));
    }

    private Command jiggle() {
        Pose here = follower.pose();
        double heading = here.heading();
        Pose anchor = p.of(here.x(), here.y(), Math.toDegrees(heading));
        Pose back = p.of(
                here.x() - GARDEN_JIGGLE_INCHES * Math.cos(heading),
                here.y() - GARDEN_JIGGLE_INCHES * Math.sin(heading),
                Math.toDegrees(heading));
        return repeat(
                sequential(
                        race(follow(follower, line(anchor, back).linear(anchor, back)),
                                waitMs(GARDEN_JIGGLE_HALF_MS)),
                        race(follow(follower, line(back, anchor).linear(back, anchor)),
                                waitMs(GARDEN_JIGGLE_HALF_MS))),
                GARDEN_JIGGLE_TIMES);
    }

    private long msLeft() {
        return AUTO_LENGTH_MS - (System.currentTimeMillis() - startTime);
    }

    // Drive straight from wherever the robot is to park. Built when it starts.
    private Command parkFromHere(String why) {
        return lazy(() -> {
            parking = true;
            phase = "parking (" + why + ")";
            Pose here = follower.pose();
            Pose from = p.of(here.x(), here.y(), Math.toDegrees(here.heading()));
            return follow(follower, line(from, park).linear(from, park));
        });
    }

    // Spin the launcher up and feed for SHOOT_MS (loop() feeds only once the launcher is up to
    // speed), then stop the launcher and the windmill. Stops early if needed so at least
    // PARK_NOW_MS is left to park.
    private Command shoot(String label) {
        return sequential(
                instant(() -> {
                    phase = label;
                    launcher.setVelocity(autoLauncherTargetVelocity());
                    launching = true;
                }),
                lazy(() -> waitMs(Math.max(0, Math.min(SHOOT_MS, msLeft() - PARK_NOW_MS)))),
                instant(() -> {
                    launching = false;
                    launcher.setVelocity(0);
                    windmillFeeder.stop();
                }));
    }

    // The whole auto. Each step starts as soon as the previous one finishes.
    private Command autoRoutine() {
        // With forward = 0 and SHOOT_HEADING = 0 we already start at shootPose, so don't
        // build or follow a zero-length path.
        Command driveToShoot = samePose(startPose, shootPose)
                ? instant(() -> { })
                : follow(follower, startToShoot());
        return sequential(
                instant(() -> phase = "driving to shootPose"),
                driveToShoot,
                instant(() -> phase = "waiting for AprilTag " + WAIT_TAG_MIN + "-" + WAIT_TAG_MAX),
                waitUntil(() -> seenTagId >= 0),
                lazy(() -> shoot("first shot (tag " + seenTagId + ")")),
                // Garden trip only if there's still SKIP_GARDEN_MS left; otherwise park now.
                lazy(() -> msLeft() < SKIP_GARDEN_MS
                        ? parkFromHere("only " + msLeft() + "ms left, skipping garden")
                        : gardenThenThirdShot()),
                instant(() -> phase = "parked"));
    }

    private Command gardenThenThirdShot() {
        return sequential(
                instant(() -> phase = "driving to garden approach"),
                follow(follower, toGardenAproach()),
                instant(() -> phase = "driving into garden"),
                gardenAndJiggle(),
                instant(() -> {
                    phase = "driving to shootPose for third shot";
                    // Spin up on the way so the launcher is closer to speed on arrival.
                    launcher.setVelocity(autoLauncherTargetVelocity());
                }),
                follow(follower, thirdShot()),
                shoot("third shot"),
                instant(() -> {
                    parking = true;
                    phase = "parking";
                }),
                follow(follower, parking()));
    }

    // Records the first visible tag in WAIT_TAG_MIN..WAIT_TAG_MAX.
    private void updateTags() {
        if (seenTagId >= 0) {
            return;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id >= WAIT_TAG_MIN && id <= WAIT_TAG_MAX) {
                seenTagId = id;
                return;
            }
        }
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
        limelight.stop();
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
        // Flower setup, copied from DriveTest; start at the base (home) position.
        rightFlowerIntakeServo = hardwareMap.get(Servo.class, "flowery");
        leftflowerIntakeServo = hardwareMap.get(Servo.class, "flowey");
        rightFlowerIntakeServo.setDirection(Servo.Direction.REVERSE);
        leftflowerIntakeServo.setDirection(Servo.Direction.FORWARD);
        rightFlowerIntakeServo.setPosition(flowerHome());
        leftflowerIntakeServo.setPosition(flowerHome());
        launcher =hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);
        leftIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        // Limelight, same setup as AutoShootBASE.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        int speed = autoLauncherTargetVelocity();
        if (speed < 100) {
            throw new RuntimeException("wrong speed " + speed);
        }
        // Spin up now so the launcher is ready by the time the tag shows up.
        launcher.setVelocity(speed);
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        intake.setPower(1);
        leftIntakeServo.setPower(1);
        rightIntakeServo.setPower(1);

        follower.update();
        updateTags();
        // Less than PARK_NOW_MS left (tag or no tag): drop whatever is running and park.
        if (!parking && msLeft() < PARK_NOW_MS) {
            launching = false;
            launcher.setVelocity(0);
            windmillFeeder.stop();
            Scheduler.reset();  // cancel whatever command is still running
            follower.stop();    // and stop the drivetrain from following its path
            schedule(sequential(
                    parkFromHere("only " + msLeft() + "ms left"),
                    instant(() -> phase = "parked")));
        }
        Scheduler.execute();

        long elapsed = System.currentTimeMillis() - startTime;
        telemetry.addData("Phase", phase);
        telemetry.addData("Time (s)", "%.1f (%.1f left)", elapsed / 1000.0, msLeft() / 1000.0);
        telemetry.addData("Tag", seenTagId >= 0 ? String.valueOf(seenTagId) : "none yet");
        telemetry.addData("Launcher", "target %d, min %d, actual %.0f",
                autoLauncherTargetVelocity(), autoLauncherMinVelocity(), launcher.getVelocity());
        telemetry.addData("Shoot heading (deg)", "using %d (dashboard now %d)",
                shootHeadingUsed, SHOOT_HEADING);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());

        if (launching) {
            if (launcher.getVelocity() >= autoLauncherMinVelocity()) {
                telemetry.addData("launch", "launch_speed_reached");
                windmillFeeder.feed();
                intakePower += 0.5;
            } else {
                telemetry.addData("launch", "launch_speed_not_reached");
                windmillFeeder.stop();
            }
        }
        telemetry.update();
    }
}
