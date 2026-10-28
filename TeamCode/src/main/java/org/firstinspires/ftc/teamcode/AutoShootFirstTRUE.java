package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.LauncherConfig.launcherMinVelocity;
import static org.firstinspires.ftc.teamcode.LauncherConfig.launcherTargetVelocity;

@Autonomous(name = "AutoShootFirstTRUE", group = "OpMode")
public class AutoShootFirstTRUE extends OpMode {

    private long startTime;

    boolean launch_Done = false;
    boolean stopped = false;
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(144, 60, 0);
    private final Pose seccondPose = p.of(130, 40, 0);
    private final Pose park = p.of(56, 20, 0);

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
    private Path startToSeccond() {

        return line(startPose, seccondPose).linear(startPose, seccondPose);
    }
    private Path park() {
        return line(seccondPose, park).linear(seccondPose, park);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToSeccond())
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
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);

    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        launcher.setVelocity(launcherTargetVelocity());
        intake.setPower(1);
    }

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());

        if (launch_Done || System.currentTimeMillis() - startTime > 10_000) {
            launcher.setVelocity(0);
            windmillServo.setPower(0);
            launcher.setPower(0);
            intake.setPower(0);
            if (!launch_Done) {
                launch_Done = true;
                telemetry.addData("launch done", "set launch done");
                intake.setPower(1);
                schedule(autoRoutine());
            }
            // Checked every loop (not just once above) so it actually fires at the 20s mark.
            if (!stopped && System.currentTimeMillis() - startTime > 13_000) {
                stopped = true;
                Scheduler.reset();  // cancel whatever command is still running
                follower.stop();    // and stop the drivetrain from following its path
                schedule(follow(follower, park()));
                telemetry.addData("auto", "stopped at 20s, driving to moveRight");
            }
            return;
        }

        if (!launch_Done) {
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

