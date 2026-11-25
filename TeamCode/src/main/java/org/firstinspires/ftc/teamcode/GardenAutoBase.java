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

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
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
 * Shared part of the garden autos (AutoShootGarden, AutoShootFirstTRUE):
 *   0-6s  drive to shootPose and shoot the preloads
 *   6s    drive to the garden approach
 *   7s    drive into the garden and jiggle
 * After 6s, every loop also calls afterGarden(), where each subclass decides what to do
 * next (shoot again from a second pose, or park).
 * Tunables (SHOOT_HEADING, GARDEN_*) are in GardenConfig, tunable on FTC Dashboard.
 */
public abstract class GardenAutoBase extends OpMode {
    protected long startTime;
    private int shootHeadingUsed = SHOOT_HEADING;

    private boolean launch_Done = false;
    private boolean stopped = false;
    protected Follower follower;
    protected final PoseFactory p = PoseFactory.degrees();
    private final Pose shootPose = p.of(144-forward, 48, shootHeadingUsed);

    private final Pose startPose = p.of(144, 48, 0);
    protected final Pose park = p.of(GARDEN_PARK_X, GARDEN_PARK_Y, 0);
    protected final Pose gardenPose = p.of(144,0,0);
    private final Pose gardenAproach = p.of(130,0,0);

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

    // Which step of the auto we're in; shown every loop.
    protected String phase = "waiting for start";

    private GoBildaPinpointDriver odo;

    protected WindmillFeeder windmillFeeder;

    /**
     * Called every loop once the 6s shooting window is over, after the shared garden
     * steps. elapsedMs is the time since PLAY.
     */
    protected abstract void afterGarden(long elapsedMs);

    private Path toGardenAproach(){
        return line(shootPose,gardenAproach).linear(shootPose, gardenAproach);
    }
    private Path toGarden(){
        return line(gardenAproach,gardenPose).linear(gardenAproach,gardenPose);
    }
    private Path startToShoot() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }

    private static boolean samePose(Pose a, Pose b) {
        return a.x() == b.x() && a.y() == b.y() && a.heading() == b.heading();
    }

    // Cancel whatever command is running, stop the drivetrain, and start the given command.
    protected void switchTo(Command command) {
        Scheduler.reset();  // cancel whatever command is still running
        follower.stop();    // and stop the drivetrain from following its path
        schedule(command);
    }

    // Drive into the garden, then rock back and forth around wherever the robot stopped.
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

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
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
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        // With forward = 0 and SHOOT_HEADING = 0 we already start at shootPose, so don't
        // build or follow a zero-length path.
        if (!samePose(startPose, shootPose)) {
            schedule(follow(follower, startToShoot()));
        }
        int speed = autoLauncherTargetVelocity();
        if (speed < 100) {
            throw new RuntimeException("wrong speed " + speed);
        }
        launcher.setVelocity(speed);
        phase = "driving to shootPose: " + speed + ":" + shootPose;
    }

    @Override
    public void loop() {
        intake.setPower(1);
        leftIntakeServo.setPower(1);
        rightIntakeServo.setPower(1);

        follower.update();
        Scheduler.execute();

        long elapsed = System.currentTimeMillis() - startTime;
        telemetry.addData("Phase", phase);
        telemetry.addData("Time (s)", "%.1f", elapsed / 1000.0);
        telemetry.addData("Launcher", "target %d, min %d, actual %.0f",
                autoLauncherTargetVelocity(), autoLauncherMinVelocity(), launcher.getVelocity());
        telemetry.addData("Shoot heading (deg)", "using %d (dashboard now %d)",
                shootHeadingUsed, SHOOT_HEADING);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());

        if (elapsed > 6_000) {
            launcher.setVelocity(0);
            neoWindmill.setPower(0);
            launcher.setPower(0);
            if (!launch_Done) {
                launch_Done = true;
                phase = "driving to garden approach";
                schedule(follow(follower, toGardenAproach()));
            }
            // Checked every loop (not just once above) so it actually fires on time.
            if (!stopped && elapsed > 7_000) {
                stopped = true;
                phase = "driving into garden";
                Constants.foresightConfig.maxPathSpeed.set(1.0);
                switchTo(gardenAndJiggle());
                Constants.foresightConfig.maxPathSpeed.set(20.0);
            }
            afterGarden(elapsed);
            // Without this the Driver Station screen froze at the 10s mark.
            telemetry.update();
            return;
        }

        if (!launch_Done) {
            if (launcher.getVelocity() >= autoLauncherMinVelocity()) {
                phase = "shooting";
                telemetry.addData("launch done", "launch_speed_reached");
                windmillFeeder.feed();
                intakePower += 0.5;
            } else {
                telemetry.addData("launch done", "launch_speed_not_reached");
                windmillFeeder.stop();
            }
        }
        telemetry.update();
    }
}
