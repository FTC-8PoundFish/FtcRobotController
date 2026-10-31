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

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.config.LauncherConfig.autoLauncherTargetVelocity;
import static org.firstinspires.ftc.config.LauncherConfig.launcherMinVelocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * Launcher speeds come from LauncherConfig (tunable on FTC Dashboard).
 */
@Autonomous(name = "AutoShootFirst", group = "OpMode")
public class AutoShootFirst extends OpMode {

    private long startTime;

    private boolean launching = false;
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(144, 60, 0);

    // move forward for 4 inches to shoot
    private static final int forward = 4;
    private final Pose shootPose = p.of(144 - forward, 60, 0);

    private final Pose seccondPose = p.of(124, 40, 0);
    private final Pose park = p.of(60, 20, 0);

    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    double intakePower;

    private GoBildaPinpointDriver odo;

    private WindmillFeeder windmillFeeder;

    // Set by each step of autoRoutine() and shown every loop.
    private String autoStage = "waiting for start";
    private CRServo windmillServo = null;

    //poses from before
    // other poses...
    private Path startToShoot() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }
    private Path shootToSeccond() {
        return line(shootPose, seccondPose).linear(shootPose, seccondPose);
    }
    private Path park() {
        return line(seccondPose, park).linear(seccondPose, park);
    }

    // The whole auto. Each step starts as soon as the previous one finishes.
    private Command autoRoutine() {
        return sequential(
                instant(() -> autoStage = "driving to shootPose"),
                follow(follower, startToShoot()),
                instant(() -> {
                    autoStage = "launching with speed:" + autoLauncherTargetVelocity();
                    launcher.setVelocity(autoLauncherTargetVelocity());
                    intake.setPower(1);
                    launching = true;  // loop() feeds the windmill while this is true
                }),
                waitMs(10_000),
                instant(() -> {
                    launching = false;
                    launcher.setVelocity(0);
                    windmillFeeder.stop();
                    intake.setPower(0);
                    autoStage = "driving to seccondPose";
                }),
                follow(follower, shootToSeccond()),
                instant(() -> autoStage = "at seccondPose, waiting 2s"),
                waitMs(2000),
                instant(() -> autoStage = "parking"),
                follow(follower, park()),
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
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);

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
            if (launcher.getVelocity() > launcherMinVelocity()) {
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

