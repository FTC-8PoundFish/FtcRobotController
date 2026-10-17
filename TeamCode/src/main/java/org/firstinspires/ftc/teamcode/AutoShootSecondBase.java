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

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

/*
 * Shared Auto for the AutoShootSecond* OpModes. Each subclass picks the launcher target velocity;
 * the minimum (feed) velocity is always target - 25.
 */
public abstract class AutoShootSecondBase extends OpMode {

    private final int launcherTargetVelocity;
    private final int launcherMinVelocity;

    protected AutoShootSecondBase(int launcherTargetVelocity) {
        this.launcherTargetVelocity = launcherTargetVelocity;
        this.launcherMinVelocity = launcherTargetVelocity - 25;
    }

    private long startTime;

    private boolean launching = false;
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(144, 84, 0);
    // move forward this many inches to shoot (same as AutoShootFirstBase)
    private static final int forward = 8;
    private final Pose shootPose = p.of(144 - forward, 84, 0);
    private final Pose park = p.of(130, 138, 0);

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


    private Path startToShoot() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }
    private Path Park() {
        return line(shootPose, park).linear(shootPose, park);
    }


    // The whole auto. Each step starts as soon as the previous one finishes.
    private Command autoRoutine() {
        return sequential(
                instant(() -> autoStage = "waiting until 10s"),
                waitMs(10_000),
                instant(() -> autoStage = "driving to shootPose"),
                follow(follower, startToShoot()),
                instant(() -> {
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
                follow(follower, Park()),
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
    }

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
        telemetry.addData(
                "Speed",
                "target (%d), min (%d)",
                launcherTargetVelocity,
                launcherMinVelocity);
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

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Auto stage", autoStage);
        telemetry.addData("Time (s)", "%.1f", (System.currentTimeMillis() - startTime) / 1000.0);

        if (launching) {
            if (launcher.getVelocity() > launcherMinVelocity) {
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

