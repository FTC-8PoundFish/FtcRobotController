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

/*
 * Shared Auto for the AutoShootSecond* OpModes. Each subclass picks the launcher target velocity
 * and how far below it the minimum (feed) velocity sits.
 */
public abstract class AutoShootSecondBase extends OpMode {

    private final int launcherTargetVelocity;
    private final int launcherMinVelocity;

    protected AutoShootSecondBase(int launcherTargetVelocity, int minVelocityGap) {
        this.launcherTargetVelocity = launcherTargetVelocity;
        this.launcherMinVelocity = launcherTargetVelocity - minVelocityGap;
    }

    private long startTime;

    boolean launch_Done = false;
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(144, 84, 0);
    private final Pose gardenPose = p.of(0, 0, 0);
    private final Pose park = p.of(130, 124, 0);

    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    double intakePower;

    private GoBildaPinpointDriver odo;

    private WindmillFeeder windmillFeeder;

    public static int Launcher_Velocity = 0;

    //poses from before
    // other poses...
    private Path startToGarden() {

        return line(startPose, gardenPose).linear(startPose, gardenPose);
    }
    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, park())
                // Add mechanism commands here.
//                follow(follower, park())
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
    }

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());

        if (launch_Done || System.currentTimeMillis() - startTime > 20_000) {
            launcher.setVelocity(0);
            windmillServo.setPower(0);
            launcher.setPower(0);
            if (!launch_Done) {
                launch_Done = true;
                telemetry.addData("launch done", "set launch done");
                schedule(autoRoutine());
            }
            return;
        }

        if (!launch_Done&& System.currentTimeMillis() - startTime > 10_000) {
            intake.setPower(1);
            launcher.setVelocity(launcherTargetVelocity);
            if (launcher.getVelocity() > launcherMinVelocity) {
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

